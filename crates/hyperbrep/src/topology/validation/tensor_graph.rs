//! Validated construction: solids capped by a rational Bézier tensor graph.

use crate::topology::*;

impl ArenaModelBuilder {
    /// A closed shell of exactly one rational Bézier face (the top), one
    /// planar base face without holes, and planar walls parallel to the base's
    /// inward normal `d`. The top must be certified as a graph strictly above
    /// the base along `d`. Since the walls are parallel to `d` and the shell is
    /// closed and oriented, the top's projection is exactly the base region
    /// and the solid is the region between them.
    pub(in crate::topology) fn certified_tensor_graph_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedTensorGraphShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut top = None;
        let mut planes = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::RationalBezier if top.is_none() => top = Some(*face_id),
                SurfaceKind::Plane => planes.push(*face_id),
                _ => return Ok(None),
            }
        }
        let Some(top) = top else {
            return Ok(None);
        };
        let top_surface = self.surface_ref(self.face_ref(top)?.surface)?.clone();
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
                Orientation::Forward => -normal,
                Orientation::Reversed => normal,
            };
            let mut walls_parallel = true;
            for wall_id in &planes {
                if wall_id == base_id {
                    continue;
                }
                let SurfaceExactData::Plane { u: wu, v: wv, .. } = self
                    .surface_ref(self.face_ref(*wall_id)?.surface)?
                    .exact_data()
                else {
                    walls_parallel = false;
                    break;
                };
                if decided_model_order(compare_reals(
                    &wu.cross(&wv).dot(&direction),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Equal
                {
                    walls_parallel = false;
                    break;
                }
            }
            if !walls_parallel || !top_surface.certify_tensor_graph(&origin, &u, &v, &direction)? {
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
            }));
        }
        Ok(None)
    }
}
