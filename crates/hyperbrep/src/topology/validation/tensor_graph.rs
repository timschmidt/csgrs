//! Validated construction: solids capped by a rational Bézier tensor graph.

use crate::topology::*;

impl ArenaModelBuilder {
    /// A closed shell of rational Bézier top faces on one tensor, one planar
    /// base face without holes, planar walls parallel to the base's inward
    /// normal `d`, and optional planar caps parallel to the base. The tensor
    /// must be certified as a graph strictly above the base along `d`.
    ///
    /// With one tensor face and no caps, the walls being parallel to `d` and
    /// the shell being closed and oriented make the top's projection exactly
    /// the base region. Otherwise every top face must also face along `d`
    /// (caps by their constant normal, tensor faces by the sign of the
    /// certified graph's normal, constant because its projection is
    /// injective), and each cap must lie strictly above the base: a vertical
    /// line over the profile then leaves the solid through exactly one top
    /// face, so the tops' regions (caps minus their holes) tile the profile.
    pub(in crate::topology) fn certified_tensor_graph_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedTensorGraphShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut tensors = Vec::new();
        let mut planes = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::RationalBezier => tensors.push(*face_id),
                SurfaceKind::Plane => planes.push(*face_id),
                _ => return Ok(None),
            }
        }
        let Some(first) = tensors.first() else {
            return Ok(None);
        };
        let top_surface = self.surface_ref(self.face_ref(*first)?.surface)?.clone();
        for face_id in &tensors[1..] {
            let surface = self.surface_ref(self.face_ref(*face_id)?.surface)?;
            if !top_surface.same_rational_tensor(surface)? {
                return Ok(None);
            }
        }
        let zero = Real::zero();
        let order = |value: &Real| {
            decided_model_order(compare_reals(value, &zero, crate::STRICT_PREDICATES))
        };
        for base_id in &planes {
            let base = self.face_ref(*base_id)?;
            let Some(outer) = base.outer() else {
                continue;
            };
            if !base.inner().is_empty() {
                continue;
            }
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(base.surface)?.exact_data()
            else {
                continue;
            };
            // The base's outward normal points away from the material.
            let normal = u.cross(&v);
            let direction = match base.orientation {
                Orientation::Forward => -normal.clone(),
                Orientation::Reversed => normal.clone(),
            };
            let determinant = u.dot(&v.cross(&direction));
            let mut admissible = true;
            let mut caps = Vec::new();
            for other_id in &planes {
                if other_id == base_id {
                    continue;
                }
                let other = self.face_ref(*other_id)?;
                let SurfaceExactData::Plane {
                    origin: cap_origin,
                    u: cap_u,
                    v: cap_v,
                } = self.surface_ref(other.surface)?.exact_data()
                else {
                    admissible = false;
                    break;
                };
                let other_normal = cap_u.cross(&cap_v);
                if order(&other_normal.dot(&direction))? == std::cmp::Ordering::Equal {
                    // A wall parallel to `d`.
                    continue;
                }
                // Otherwise a cap: parallel to the base, facing along `d`,
                // strictly above the base and without holes.
                let parallel = other_normal.cross(&normal);
                let outward = match other.orientation {
                    Orientation::Forward => other_normal,
                    Orientation::Reversed => -other_normal,
                };
                let height = ((&cap_origin - &origin).dot(&u.cross(&v)) / &determinant)
                    .map_err(GeometryError::Numeric)?;
                let Some(cap_outer) = other.outer() else {
                    admissible = false;
                    break;
                };
                if order(&parallel.dot(&parallel))? != std::cmp::Ordering::Equal
                    || order(&outward.dot(&direction))? != std::cmp::Ordering::Greater
                    || order(&height)? != std::cmp::Ordering::Greater
                {
                    admissible = false;
                    break;
                }
                caps.push(CertifiedGraphCap {
                    origin: cap_origin,
                    u: cap_u,
                    v: cap_v,
                    height,
                    boundary: self.build_wire_curve_path(cap_outer)?,
                    holes: other
                        .inner()
                        .iter()
                        .map(|wire| self.build_wire_curve_path(*wire))
                        .collect::<Result<Vec<_>, _>>()?,
                });
            }
            if !admissible || !top_surface.certify_tensor_graph(&origin, &u, &v, &direction)? {
                continue;
            }
            if (tensors.len() > 1 || !caps.is_empty())
                && !self.tensor_faces_face_along(&tensors, &top_surface, &direction)?
            {
                continue;
            }
            let profile = self.build_wire_curve_path(outer)?;
            return Ok(Some(CertifiedTensorGraphShell {
                origin,
                u,
                v,
                direction,
                profile,
                top: top_surface,
                caps,
            }));
        }
        Ok(None)
    }

    /// Whether every tensor face's outward normal points along `direction`.
    /// The certified graph's projection is injective, so its normal has one
    /// sign against `direction` across the patch; the patch centre decides it.
    fn tensor_faces_face_along(
        &self,
        tensors: &[FaceId],
        surface: &Surface,
        direction: &Vector3,
    ) -> Result<bool, BuildError> {
        let half = (Real::one() / Real::from(2)).map_err(GeometryError::Numeric)?;
        let normal = match surface.normal_at(&CurvePoint2::new(half.clone(), half).into()) {
            Ok(normal) => normal,
            Err(GeometryError::SingularSurfaceParameter) => return Ok(false),
            Err(error) => return Err(error.into()),
        };
        let sign = normal.compare_linear_form(direction, &Real::zero())?;
        if sign == std::cmp::Ordering::Equal {
            return Ok(false);
        }
        for face_id in tensors {
            let along = match self.face_ref(*face_id)?.orientation {
                Orientation::Forward => sign,
                Orientation::Reversed => sign.reverse(),
            };
            if along != std::cmp::Ordering::Greater {
                return Ok(false);
            }
        }
        Ok(true)
    }
}
