//! Validated construction: validation sphere_cylinder.

use crate::topology::*;

impl ModelBuilder {
    pub(in crate::topology) fn certified_sphere_cylinder_capped_shell(
        &self,
        shell: ShellId,
        faces: &[FaceId],
        sphere_faces: &[FaceId],
        cylinder_faces: &[FaceId],
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let [sphere_face] = sphere_faces else {
            return Ok(None);
        };
        let Some(sphere_cap) = self.certified_spherical_cap_face(*sphere_face)? else {
            return Ok(None);
        };
        if sphere_cap.orientation != Orientation::Forward {
            return Ok(None);
        }
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
            axis,
            radius: cylinder_radius,
            ..
        } = self.surface_ref(cylinder_surface_id)?.exact_data()
        else {
            return Ok(None);
        };
        if !vectors_equal(&sphere_cap.axis, &axis)?
            || decided_model_order(compare_reals(
                &cylinder_radius,
                &sphere_cap.radius,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let center_offset = &sphere_cap.center - &origin;
        let center_parameter = center_offset.dot(&axis);
        let radial_offset = center_offset - axis.clone() * &center_parameter;
        if decided_model_order(compare_reals(
            &radial_offset.norm_squared(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }
        let intersection_height = &sphere_cap.radius * sphere_cap.latitude.clone().sin();
        let height_order = decided_model_order(compare_reals(
            &intersection_height,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))?;
        if height_order == std::cmp::Ordering::Equal
            || !real_values_equal(
                &(&sphere_cap.radius * sphere_cap.latitude.clone().cos()),
                &cylinder_radius,
            )?
        {
            return Ok(None);
        }
        let intersection_parameter = &center_parameter + &intersection_height;
        let (orientation, cylinder) = match (
            self.certified_cylinder_side_faces(shell, cylinder_faces, Orientation::Forward)?,
            self.certified_cylinder_side_faces(shell, cylinder_faces, Orientation::Reversed)?,
        ) {
            (Some(cylinder), None) => (Orientation::Forward, cylinder),
            (None, Some(cylinder)) => (Orientation::Reversed, cylinder),
            _ => return Ok(None),
        };
        let intersection_at_min = real_values_equal(&cylinder.v_min, &intersection_parameter)?;
        let intersection_at_max = real_values_equal(&cylinder.v_max, &intersection_parameter)?;
        if intersection_at_min == intersection_at_max {
            return Ok(None);
        }
        let extends_positive = intersection_at_min;
        let extends_toward_center = match height_order {
            std::cmp::Ordering::Less => extends_positive,
            std::cmp::Ordering::Greater => !extends_positive,
            std::cmp::Ordering::Equal => unreachable!("zero height was rejected"),
        };
        let operation = match (orientation, extends_toward_center) {
            (Orientation::Forward, true) => CertifiedSphereFiniteCylinderOperation::Intersection,
            (Orientation::Forward, false) => CertifiedSphereFiniteCylinderOperation::Union,
            (Orientation::Reversed, true) => CertifiedSphereFiniteCylinderOperation::Difference,
            (Orientation::Reversed, false) => return Ok(None),
        };
        let sphere_retains_intersection_side =
            sphere_cap.upper == (height_order == std::cmp::Ordering::Greater);
        if sphere_retains_intersection_side
            != (operation == CertifiedSphereFiniteCylinderOperation::Intersection)
        {
            return Ok(None);
        }
        let cap_groups = self.planar_face_groups(faces)?;
        let [cap_group] = cap_groups.as_slice() else {
            return Ok(None);
        };
        let cap_parameter = if intersection_at_min {
            cylinder.v_max.clone()
        } else {
            cylinder.v_min.clone()
        };
        let ordinary_cap_direction = if intersection_at_min {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Less
        };
        let expected_cap_direction = if orientation == Orientation::Forward {
            ordinary_cap_direction
        } else {
            ordinary_cap_direction.reverse()
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
        let relative_cap = &cap_parameter - &center_parameter;
        match operation {
            CertifiedSphereFiniteCylinderOperation::Union => {
                let outside_pole = if extends_positive {
                    decided_model_order(compare_reals(
                        &relative_cap,
                        &sphere_cap.radius,
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Greater
                } else {
                    decided_model_order(compare_reals(
                        &relative_cap,
                        &-sphere_cap.radius.clone(),
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Less
                };
                if !outside_pole {
                    return Ok(None);
                }
            }
            CertifiedSphereFiniteCylinderOperation::Intersection
            | CertifiedSphereFiniteCylinderOperation::Difference => {
                let half_height = (&sphere_cap.radius * &sphere_cap.radius
                    - &cylinder_radius * &cylinder_radius)
                    .sqrt()
                    .map_err(GeometryError::Numeric)?;
                if decided_model_order(compare_reals(
                    &relative_cap,
                    &-half_height.clone(),
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Greater
                    || decided_model_order(compare_reals(
                        &relative_cap,
                        &half_height,
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Less
                {
                    return Ok(None);
                }
            }
        }
        let (region_min, region_max) = match operation {
            CertifiedSphereFiniteCylinderOperation::Union => (cylinder.v_min, cylinder.v_max),
            CertifiedSphereFiniteCylinderOperation::Intersection
            | CertifiedSphereFiniteCylinderOperation::Difference => {
                if height_order == std::cmp::Ordering::Less {
                    (
                        &center_parameter - &sphere_cap.radius - &cylinder.radius,
                        cap_parameter.clone(),
                    )
                } else {
                    (
                        cap_parameter.clone(),
                        &center_parameter + &sphere_cap.radius + &cylinder.radius,
                    )
                }
            }
        };
        Ok(Some(CertifiedSphereShell {
            center: sphere_cap.center,
            radius: sphere_cap.radius,
            voids: Vec::new(),
            region: CertifiedSphereRegion::FiniteCylinder(CertifiedSphereFiniteCylinderRegion {
                origin: cylinder.origin,
                axis: cylinder.axis,
                radius: cylinder.radius,
                v_min: region_min,
                v_max: region_max,
                operation,
            }),
        }))
    }
}
