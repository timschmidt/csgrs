//! Validated construction: validation sphere.

use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn certified_spherical_cap_face(
        &self,
        face_id: FaceId,
    ) -> Result<Option<CertifiedSphericalCapFace>, BuildError> {
        let face = self.face_ref(face_id)?;
        let Some(outer) = face.outer() else {
            return Ok(None);
        };
        if !face.inner().is_empty() {
            return Ok(None);
        }
        let SurfaceExactData::Sphere {
            center,
            axis,
            radius,
            ..
        } = self.surface_ref(face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let wire = self.wire_ref(outer)?;
        let Some(first_use) = wire.edge_uses.first() else {
            return Ok(None);
        };
        let pcurve = self.pcurve_ref(self.edge_use_ref(*first_use)?.pcurve)?;
        let Some(line) = pcurve.line_segment() else {
            return Ok(None);
        };
        let increasing = decided_model_order(compare_reals(
            line.end().x(),
            line.start().x(),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Greater;
        let upper = match face.orientation {
            Orientation::Forward => increasing,
            Orientation::Reversed => !increasing,
        };
        Ok(Some(CertifiedSphericalCapFace {
            center,
            axis,
            radius,
            latitude: line.start().y().clone(),
            upper,
            orientation: face.orientation,
        }))
    }

    pub(in crate::topology) fn certified_sphere_pair_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedSpherePairShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() != 2 {
            return Ok(None);
        }
        let Some(first) = self.certified_spherical_cap_face(faces[0])? else {
            return Ok(None);
        };
        let Some(second) = self.certified_spherical_cap_face(faces[1])? else {
            return Ok(None);
        };
        let displacement = &second.center - &first.center;
        let distance_squared = displacement.norm_squared();
        if decided_model_order(compare_reals(
            &distance_squared,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let radius_sum = &first.radius + &second.radius;
        let radius_difference = (&first.radius - &second.radius).abs();
        if decided_model_order(compare_reals(
            &distance_squared,
            &(&radius_sum * &radius_sum),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(
                &distance_squared,
                &(&radius_difference * &radius_difference),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let first_pole = if first.upper {
            first.center.clone() + first.axis.clone() * &first.radius
        } else {
            first.center.clone() - first.axis.clone() * &first.radius
        };
        let second_pole = if second.upper {
            second.center.clone() + second.axis.clone() * &second.radius
        } else {
            second.center.clone() - second.axis.clone() * &second.radius
        };
        let first_inside_second = decided_model_order(compare_reals(
            &(&first_pole - &second.center).norm_squared(),
            &(&second.radius * &second.radius),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less;
        let second_inside_first = decided_model_order(compare_reals(
            &(&second_pole - &first.center).norm_squared(),
            &(&first.radius * &first.radius),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less;

        match (first.orientation, second.orientation) {
            (Orientation::Forward, Orientation::Forward)
                if first_inside_second == second_inside_first =>
            {
                Ok(Some(CertifiedSpherePairShell {
                    first_center: first.center,
                    first_radius: first.radius,
                    second_center: second.center,
                    second_radius: second.radius,
                    kind: if first_inside_second {
                        CertifiedSpherePairKind::Intersection
                    } else {
                        CertifiedSpherePairKind::Union
                    },
                }))
            }
            (Orientation::Forward, Orientation::Reversed)
                if !first_inside_second && second_inside_first =>
            {
                Ok(Some(CertifiedSpherePairShell {
                    first_center: first.center,
                    first_radius: first.radius,
                    second_center: second.center,
                    second_radius: second.radius,
                    kind: CertifiedSpherePairKind::Difference,
                }))
            }
            (Orientation::Reversed, Orientation::Forward)
                if first_inside_second && !second_inside_first =>
            {
                Ok(Some(CertifiedSpherePairShell {
                    first_center: second.center,
                    first_radius: second.radius,
                    second_center: first.center,
                    second_radius: first.radius,
                    kind: CertifiedSpherePairKind::Difference,
                }))
            }
            _ => Ok(None),
        }
    }

    pub(in crate::topology) fn certified_sphere_solid(
        &self,
        solid: &Solid,
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        if let Some((center, radius)) =
            self.certified_oriented_sphere_shell(solid.outer, Orientation::Forward)?
        {
            let mut voids = Vec::with_capacity(solid.voids.len());
            for shell in &solid.voids {
                if let Some((void_center, void_radius)) =
                    self.certified_oriented_sphere_shell(*shell, Orientation::Reversed)?
                {
                    voids.push(CertifiedSphereVoid::Sphere {
                        center: void_center,
                        radius: void_radius,
                    });
                    continue;
                }
                if solid.voids.len() != 1 {
                    return Ok(None);
                }
                let Some(cylinder) =
                    self.certified_oriented_cylinder_shell(*shell, Orientation::Reversed)?
                else {
                    return Ok(None);
                };
                if !self.sphere_strictly_contains_cylinder(&center, &radius, &cylinder)? {
                    return Ok(None);
                }
                voids.push(CertifiedSphereVoid::Cylinder(Box::new(
                    CertifiedSphereCylinderVoid {
                        origin: cylinder.origin,
                        axis: cylinder.axis,
                        radius: cylinder.radius,
                        v_min: cylinder.v_min,
                        v_max: cylinder.v_max,
                    },
                )));
            }
            return Ok(Some(CertifiedSphereShell {
                center,
                radius,
                voids,
                region: CertifiedSphereRegion::Whole,
            }));
        }
        if !solid.voids.is_empty() {
            return Ok(None);
        }
        self.certified_sphere_segment_shell(solid.outer)
    }

    pub(in crate::topology) fn sphere_strictly_contains_cylinder(
        &self,
        sphere_center: &Point3,
        sphere_radius: &Real,
        cylinder: &CertifiedCylinderShell,
    ) -> Result<bool, BuildError> {
        if cylinder.sphere_subtraction.is_some() {
            return Ok(false);
        }
        CertifiedSphereProfile {
            center: sphere_center.clone(),
            radius: sphere_radius.clone(),
        }
        .strictly_contains_cylinder(&CertifiedCylinderProfile {
            origin: cylinder.origin.clone(),
            axis: cylinder.axis.clone(),
            radius: cylinder.radius.clone(),
            v_min: cylinder.v_min.clone(),
            v_max: cylinder.v_max.clone(),
        })
        .map_err(BuildError::from)
    }

    pub(in crate::topology) fn cylinder_strictly_contains_sphere(
        &self,
        cylinder: &CertifiedCylinderShell,
        sphere_center: &Point3,
        sphere_radius: &Real,
    ) -> Result<bool, BuildError> {
        if cylinder.sphere_subtraction.is_some() {
            return Ok(false);
        }
        CertifiedCylinderProfile {
            origin: cylinder.origin.clone(),
            axis: cylinder.axis.clone(),
            radius: cylinder.radius.clone(),
            v_min: cylinder.v_min.clone(),
            v_max: cylinder.v_max.clone(),
        }
        .strictly_contains_sphere(&CertifiedSphereProfile {
            center: sphere_center.clone(),
            radius: sphere_radius.clone(),
        })
        .map_err(BuildError::from)
    }

    pub(in crate::topology) fn certified_sphere_segment_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut sphere_faces = Vec::new();
        let mut cylinder_faces = Vec::new();
        let mut planar_faces = 0_usize;
        for face_id in faces {
            match self.surface_ref(self.face_ref(*face_id)?.surface)?.kind() {
                SurfaceKind::Plane => planar_faces += 1,
                SurfaceKind::Sphere => sphere_faces.push(*face_id),
                SurfaceKind::Cylinder => cylinder_faces.push(*face_id),
                _ => return Ok(None),
            }
        }
        if !cylinder_faces.is_empty() {
            if planar_faces == 0 {
                return self.certified_sphere_cylinder_shell(shell, &sphere_faces, &cylinder_faces);
            }
            if let Some(certificate) = self.certified_sphere_cylinder_union_shell(
                shell,
                faces,
                &sphere_faces,
                &cylinder_faces,
            )? {
                return Ok(Some(certificate));
            }
            return self.certified_sphere_cylinder_capped_shell(
                shell,
                faces,
                &sphere_faces,
                &cylinder_faces,
            );
        }
        if sphere_faces.len() != 1 {
            return Ok(None);
        }
        if !self.face_ref(sphere_faces[0])?.inner().is_empty() {
            return self.certified_sphere_band_shell(shell, sphere_faces[0]);
        }
        let Some(cap) = self.certified_spherical_cap_face(sphere_faces[0])? else {
            return Ok(None);
        };
        if cap.orientation != Orientation::Forward {
            return Ok(None);
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() != 1 {
            return Ok(None);
        }
        let height = &cap.radius * cap.latitude.clone().sin();
        let expected_center = cap.center.clone() + cap.axis.clone() * &height;
        let expected_radius = &cap.radius * cap.latitude.clone().cos();
        let Some(boundaries) = self.cap_boundary_use_loops(faces, &cap_groups[0])? else {
            return Ok(None);
        };
        if boundaries.len() != 1 {
            return Ok(None);
        }
        let mut sweep = Real::zero();
        for edge_use_id in &boundaries[0] {
            let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
            let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
            else {
                return Ok(None);
            };
            if !circle.circle
                || !points_equal(&circle.center, &expected_center)?
                || !real_values_equal(&circle.x_radius, &expected_radius)?
                || !real_values_equal(&circle.y_radius, &expected_radius)?
            {
                return Ok(None);
            }
            sweep += edge.domain.represented_end()? - edge.domain.represented_start()?;
        }
        if !real_values_equal(&sweep, &Real::tau())? {
            return Ok(None);
        }

        let expected_outward = if cap.upper {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        };
        for index in &cap_groups[0] {
            let face = self.face_ref(faces[*index])?;
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(face.surface)?.exact_data()
            else {
                return Ok(None);
            };
            let normal = u.cross(&v);
            if decided_model_order(compare_reals(
                &normal.cross(&cap.axis).norm_squared(),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
                || !real_values_equal(&(origin - &cap.center).dot(&cap.axis), &height)?
            {
                return Ok(None);
            }
            let oriented = match face.orientation {
                Orientation::Forward => normal.dot(&cap.axis),
                Orientation::Reversed => -normal.dot(&cap.axis),
            };
            if decided_model_order(compare_reals(
                &oriented,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != expected_outward
            {
                return Ok(None);
            }
        }
        Ok(Some(CertifiedSphereShell {
            center: cap.center,
            radius: cap.radius.clone(),
            voids: Vec::new(),
            region: CertifiedSphereRegion::Axial(CertifiedSphereAxialClip {
                axis: cap.axis,
                min: if cap.upper {
                    height.clone()
                } else {
                    -cap.radius.clone()
                },
                max: if cap.upper { cap.radius } else { height },
            }),
        }))
    }

    pub(in crate::topology) fn certified_sphere_band_shell(
        &self,
        shell: ShellId,
        sphere_face_id: FaceId,
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let sphere_face = self.face_ref(sphere_face_id)?;
        if sphere_face.orientation != Orientation::Forward {
            return Ok(None);
        }
        let [upper_wire] = sphere_face.inner() else {
            return Ok(None);
        };
        let Some(lower_wire) = sphere_face.outer() else {
            return Ok(None);
        };
        let (lower_latitude, _) = self.spherical_trim_coordinates(lower_wire)?;
        let (upper_latitude, _) = self.spherical_trim_coordinates(*upper_wire)?;
        let SurfaceExactData::Sphere {
            center,
            axis,
            radius,
            ..
        } = self.surface_ref(sphere_face.surface)?.exact_data()
        else {
            unreachable!("sphere face carries sphere exact data");
        };
        let expected_min = &radius * lower_latitude.sin();
        let expected_max = &radius * upper_latitude.sin();
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() != 2 {
            return Ok(None);
        }
        let mut caps = Vec::with_capacity(2);
        for group in &cap_groups {
            let Some(cap) =
                self.certified_sphere_planar_cap_group(faces, group, &center, &axis, &radius)?
            else {
                return Ok(None);
            };
            caps.push(cap);
        }
        if decided_model_order(compare_reals(
            &caps[0].0,
            &caps[1].0,
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Greater
        {
            caps.swap(0, 1);
        }
        if !real_values_equal(&caps[0].0, &expected_min)?
            || !real_values_equal(&caps[1].0, &expected_max)?
            || caps[0].1 != std::cmp::Ordering::Less
            || caps[1].1 != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        Ok(Some(CertifiedSphereShell {
            center,
            radius,
            voids: Vec::new(),
            region: CertifiedSphereRegion::Axial(CertifiedSphereAxialClip {
                axis,
                min: expected_min,
                max: expected_max,
            }),
        }))
    }

    pub(in crate::topology) fn certified_sphere_planar_cap_group(
        &self,
        faces: &[FaceId],
        group: &[usize],
        center: &Point3,
        axis: &Vector3,
        radius: &Real,
    ) -> Result<Option<(Real, std::cmp::Ordering)>, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
            return Ok(None);
        };
        if boundaries.len() != 1 {
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
        let height = (&first_circle.center - center).dot(axis);
        let expected_center = center.clone() + axis.clone() * &height;
        let expected_radius = (radius * radius - &height * &height)
            .sqrt()
            .map_err(GeometryError::Numeric)?;
        if !points_equal(&first_circle.center, &expected_center)? {
            return Ok(None);
        }
        let mut sweep = Real::zero();
        for edge_use_id in &boundaries[0] {
            let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
            let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
            else {
                return Ok(None);
            };
            if !circle.circle
                || !points_equal(&circle.center, &expected_center)?
                || !real_values_equal(&circle.x_radius, &expected_radius)?
                || !real_values_equal(&circle.y_radius, &expected_radius)?
            {
                return Ok(None);
            }
            sweep += edge.domain.represented_end()? - edge.domain.represented_start()?;
        }
        if !real_values_equal(&sweep, &Real::tau())? {
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
            if decided_model_order(compare_reals(
                &normal.cross(axis).norm_squared(),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
                || !real_values_equal(&(origin - center).dot(axis), &height)?
            {
                return Ok(None);
            }
            let oriented = match face.orientation {
                Orientation::Forward => normal.dot(axis),
                Orientation::Reversed => -normal.dot(axis),
            };
            let direction = decided_model_order(compare_reals(
                &oriented,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))?;
            if direction == std::cmp::Ordering::Equal
                || outward.is_some_and(|expected| expected != direction)
            {
                return Ok(None);
            }
            outward = Some(direction);
        }
        Ok(outward.map(|outward| (height, outward)))
    }

    pub(in crate::topology) fn certified_sphere_cylinder_shell(
        &self,
        shell: ShellId,
        sphere_faces: &[FaceId],
        cylinder_faces: &[FaceId],
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let (
            center,
            axis,
            radius,
            lower_latitude,
            upper_latitude,
            radial_side,
            cylinder_orientation,
        ) = match sphere_faces {
            [first_id, second_id] => {
                let Some(first) = self.certified_spherical_cap_face(*first_id)? else {
                    return Ok(None);
                };
                let Some(second) = self.certified_spherical_cap_face(*second_id)? else {
                    return Ok(None);
                };
                if first.orientation != Orientation::Forward
                    || second.orientation != Orientation::Forward
                    || first.upper == second.upper
                    || !points_equal(&first.center, &second.center)?
                    || !vectors_equal(&first.axis, &second.axis)?
                    || !real_values_equal(&first.radius, &second.radius)?
                {
                    return Ok(None);
                }
                let (lower, upper) = if first.upper {
                    (second, first)
                } else {
                    (first, second)
                };
                (
                    lower.center,
                    lower.axis,
                    lower.radius,
                    lower.latitude,
                    upper.latitude,
                    CertifiedSphereRadialSide::Inside,
                    Orientation::Forward,
                )
            }
            [band_id] => {
                let face = self.face_ref(*band_id)?;
                if face.orientation != Orientation::Forward {
                    return Ok(None);
                }
                let Some(lower_wire) = face.outer() else {
                    return Ok(None);
                };
                let [upper_wire] = face.inner() else {
                    return Ok(None);
                };
                let (lower_latitude, _) = self.spherical_trim_coordinates(lower_wire)?;
                let (upper_latitude, _) = self.spherical_trim_coordinates(*upper_wire)?;
                let SurfaceExactData::Sphere {
                    center,
                    axis,
                    radius,
                    ..
                } = self.surface_ref(face.surface)?.exact_data()
                else {
                    return Ok(None);
                };
                (
                    center,
                    axis,
                    radius,
                    lower_latitude,
                    upper_latitude,
                    CertifiedSphereRadialSide::Outside,
                    Orientation::Reversed,
                )
            }
            _ => return Ok(None),
        };
        let Some(cylinder) =
            self.certified_cylinder_side_faces(shell, cylinder_faces, cylinder_orientation)?
        else {
            return Ok(None);
        };
        if !vectors_equal(&axis, &cylinder.axis)?
            || decided_model_order(compare_reals(
                &cylinder.radius,
                &radius,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let center_offset = &center - &cylinder.origin;
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
        let lower_height = &radius * lower_latitude.clone().sin();
        let upper_height = &radius * upper_latitude.clone().sin();
        if decided_model_order(compare_reals(
            &lower_height,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(
                &upper_height,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
            || !real_values_equal(&lower_height, &-upper_height.clone())?
            || !real_values_equal(&(&radius * lower_latitude.clone().cos()), &cylinder.radius)?
            || !real_values_equal(&(&radius * upper_latitude.clone().cos()), &cylinder.radius)?
            || !real_values_equal(&cylinder.v_min, &(&center_parameter + &lower_height))?
            || !real_values_equal(&cylinder.v_max, &(&center_parameter + &upper_height))?
        {
            return Ok(None);
        }
        Ok(Some(CertifiedSphereShell {
            center,
            radius,
            voids: Vec::new(),
            region: CertifiedSphereRegion::Radial(CertifiedSphereRadialClip {
                axis: cylinder.axis,
                radius: cylinder.radius,
                side: radial_side,
            }),
        }))
    }

    pub(in crate::topology) fn certified_sphere_cylinder_union_shell(
        &self,
        shell: ShellId,
        faces: &[FaceId],
        sphere_faces: &[FaceId],
        cylinder_faces: &[FaceId],
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let [sphere_face_id] = sphere_faces else {
            return Ok(None);
        };
        let sphere_face = self.face_ref(*sphere_face_id)?;
        if sphere_face.orientation != Orientation::Forward {
            return Ok(None);
        }
        let Some(lower_wire) = sphere_face.outer() else {
            return Ok(None);
        };
        let [upper_wire] = sphere_face.inner() else {
            return Ok(None);
        };
        let (lower_latitude, _) = self.spherical_trim_coordinates(lower_wire)?;
        let (upper_latitude, _) = self.spherical_trim_coordinates(*upper_wire)?;
        let SurfaceExactData::Sphere {
            center,
            axis,
            radius,
            ..
        } = self.surface_ref(sphere_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let Some(first_cylinder_face) = cylinder_faces.first() else {
            return Ok(None);
        };
        let cylinder_surface_id = self.face_ref(*first_cylinder_face)?.surface;
        if cylinder_faces
            .iter()
            .any(|face| self.face_ref(*face).map(|face| face.surface) != Ok(cylinder_surface_id))
        {
            return Ok(None);
        }
        let SurfaceExactData::Cylinder {
            origin,
            axis: cylinder_axis,
            radius: cylinder_radius,
            ..
        } = self.surface_ref(cylinder_surface_id)?.exact_data()
        else {
            return Ok(None);
        };
        if !vectors_equal(&axis, &cylinder_axis)?
            || decided_model_order(compare_reals(
                &cylinder_radius,
                &radius,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let center_offset = &center - &origin;
        let center_parameter = center_offset.dot(&cylinder_axis);
        let radial_offset = center_offset - cylinder_axis.clone() * &center_parameter;
        if decided_model_order(compare_reals(
            &radial_offset.norm_squared(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }
        let lower_height = &radius * lower_latitude.clone().sin();
        let upper_height = &radius * upper_latitude.clone().sin();
        if decided_model_order(compare_reals(
            &lower_height,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(
                &upper_height,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
            || !real_values_equal(&lower_height, &-upper_height.clone())?
            || !real_values_equal(&(&radius * lower_latitude.clone().cos()), &cylinder_radius)?
            || !real_values_equal(&(&radius * upper_latitude.clone().cos()), &cylinder_radius)?
        {
            return Ok(None);
        }
        let lower_intersection = &center_parameter + &lower_height;
        let upper_intersection = &center_parameter + &upper_height;
        let mut lower_faces = Vec::new();
        let mut upper_faces = Vec::new();
        for face in cylinder_faces {
            let Some((face_min, face_max)) = self.cylinder_face_v_bounds(*face)? else {
                return Ok(None);
            };
            if decided_model_order(compare_reals(
                &face_max,
                &lower_intersection,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
            {
                lower_faces.push(*face);
            } else if decided_model_order(compare_reals(
                &face_min,
                &upper_intersection,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
            {
                upper_faces.push(*face);
            } else {
                return Ok(None);
            }
        }
        let Some(lower) =
            self.certified_cylinder_side_faces(shell, &lower_faces, Orientation::Forward)?
        else {
            return Ok(None);
        };
        let Some(upper) =
            self.certified_cylinder_side_faces(shell, &upper_faces, Orientation::Forward)?
        else {
            return Ok(None);
        };
        if !points_equal(&lower.origin, &upper.origin)?
            || !vectors_equal(&lower.axis, &upper.axis)?
            || !real_values_equal(&lower.radius, &upper.radius)?
            || !real_values_equal(&lower.v_max, &lower_intersection)?
            || !real_values_equal(&upper.v_min, &upper_intersection)?
            || decided_model_order(compare_reals(
                &(&lower.v_min - &center_parameter),
                &-radius.clone(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(
                &(&upper.v_max - &center_parameter),
                &radius,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() != 2 {
            return Ok(None);
        }
        let mut matched = [false; 2];
        for group in &cap_groups {
            let lower_match = self.certified_cylinder_cap_group(
                faces,
                group,
                &lower,
                &lower.v_min,
                std::cmp::Ordering::Less,
            )?;
            let upper_match = self.certified_cylinder_cap_group(
                faces,
                group,
                &upper,
                &upper.v_max,
                std::cmp::Ordering::Greater,
            )?;
            match (lower_match, upper_match) {
                (true, false) if !matched[0] => matched[0] = true,
                (false, true) if !matched[1] => matched[1] = true,
                _ => return Ok(None),
            }
        }
        if matched != [true, true] {
            return Ok(None);
        }
        Ok(Some(CertifiedSphereShell {
            center,
            radius,
            voids: Vec::new(),
            region: CertifiedSphereRegion::FiniteCylinder(CertifiedSphereFiniteCylinderRegion {
                origin: lower.origin,
                axis: lower.axis,
                radius: lower.radius,
                v_min: lower.v_min,
                v_max: upper.v_max,
                operation: CertifiedSphereFiniteCylinderOperation::Union,
            }),
        }))
    }

    pub(in crate::topology) fn certified_oriented_sphere_shell(
        &self,
        shell: ShellId,
        orientation: Orientation,
    ) -> Result<Option<(Point3, Real)>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        match faces.as_slice() {
            [face_id] => {
                let face = self.face_ref(*face_id)?;
                if !face.is_whole_surface() || face.orientation != orientation {
                    return Ok(None);
                }
                let SurfaceExactData::Sphere { center, radius, .. } =
                    self.surface_ref(face.surface)?.exact_data()
                else {
                    return Ok(None);
                };
                Ok(Some((center, radius)))
            }
            [first_id, second_id] => {
                let first_face = self.face_ref(*first_id)?;
                let second_face = self.face_ref(*second_id)?;
                if first_face.orientation != orientation
                    || second_face.orientation != orientation
                    || first_face.surface != second_face.surface
                {
                    return Ok(None);
                }
                let Some(first) = self.certified_spherical_cap_face(*first_id)? else {
                    return Ok(None);
                };
                let Some(second) = self.certified_spherical_cap_face(*second_id)? else {
                    return Ok(None);
                };
                if first.upper == second.upper
                    || !points_equal(&first.center, &second.center)?
                    || !vectors_equal(&first.axis, &second.axis)?
                    || !real_values_equal(&first.radius, &second.radius)?
                {
                    return Ok(None);
                }
                Ok(Some((first.center, first.radius)))
            }
            _ => self.certified_subdivided_sphere_shell(shell, orientation),
        }
    }

    pub(in crate::topology) fn certified_subdivided_sphere_shell(
        &self,
        shell: ShellId,
        orientation: Orientation,
    ) -> Result<Option<(Point3, Real)>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut surface_id = None;
        let mut lower_cap = None;
        let mut upper_cap = None;
        let mut bands: Vec<(Real, Real)> = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if face.orientation != orientation
                || self.surface_ref(face.surface)?.kind() != SurfaceKind::Sphere
            {
                return Ok(None);
            }
            match surface_id {
                Some(expected) if expected != face.surface => return Ok(None),
                Some(_) => {}
                None => surface_id = Some(face.surface),
            }
            if face.inner().is_empty() {
                let Some(cap) = self.certified_spherical_cap_face(*face_id)? else {
                    return Ok(None);
                };
                let slot = if cap.upper {
                    &mut upper_cap
                } else {
                    &mut lower_cap
                };
                if slot.replace(cap).is_some() {
                    return Ok(None);
                }
            } else if let [upper_wire] = face.inner() {
                let Some(lower_wire) = face.outer() else {
                    return Ok(None);
                };
                let (lower, _) = self.spherical_trim_coordinates(lower_wire)?;
                let (upper, _) = self.spherical_trim_coordinates(*upper_wire)?;
                if decided_model_order(compare_reals(&lower, &upper, crate::STRICT_PREDICATES))?
                    != std::cmp::Ordering::Less
                {
                    return Ok(None);
                }
                let mut insertion = bands.len();
                while insertion > 0
                    && decided_model_order(compare_reals(
                        &lower,
                        &bands[insertion - 1].0,
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Less
                {
                    insertion -= 1;
                }
                bands.insert(insertion, (lower, upper));
            } else {
                return Ok(None);
            }
        }
        let (Some(lower_cap), Some(upper_cap)) = (lower_cap, upper_cap) else {
            return Ok(None);
        };
        if bands.is_empty()
            || !points_equal(&lower_cap.center, &upper_cap.center)?
            || !vectors_equal(&lower_cap.axis, &upper_cap.axis)?
            || !real_values_equal(&lower_cap.radius, &upper_cap.radius)?
            || !real_values_equal(&bands[0].0, &lower_cap.latitude)?
            || !real_values_equal(
                &bands.last().expect("nonempty sphere bands").1,
                &upper_cap.latitude,
            )?
        {
            return Ok(None);
        }
        for pair in bands.windows(2) {
            if !real_values_equal(&pair[0].1, &pair[1].0)? {
                return Ok(None);
            }
        }
        Ok(Some((lower_cap.center, lower_cap.radius)))
    }
}
