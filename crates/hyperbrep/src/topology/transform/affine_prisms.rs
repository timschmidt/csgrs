//! Exact representation transport for cylinder walls of affine prism images.
use super::*;

impl ArenaModel {
    /// Native cylinders require similarity metrics; their full finite face
    /// images also have an exact extrusion representation. A retained prism
    /// proof permits arbitrary invertible affine transport of that material.
    /// Keep its original angular chart and derive the finite domain from all
    /// actual pcurve bounds, including distant authored periodic lifts.
    pub(super) fn cylinder_prisms_as_extrusions(&self) -> Result<Self, GeometryError> {
        let mut model = self.clone();
        let data = Arc::make_mut(&mut model.data);
        for (index, solid) in self.data.solids.iter().enumerate() {
            let has_cylinder = std::iter::once(solid.outer)
                .chain(solid.voids.iter().copied())
                .flat_map(|shell| &self.data.shells[shell.index()].faces)
                .any(|face| {
                    self.data.surfaces[self.data.faces[face.index()].surface.index()].kind()
                        == SurfaceKind::Cylinder
                });
            if has_cylinder {
                if self.data.certified_prisms[index].is_none() {
                    return Err(GeometryError::UnsupportedTransform);
                }
                data.certified_cylinders[index] = None;
                data.certified_revolutions[index] = None;
            }
        }
        for (index, surface) in self.data.surfaces.iter().enumerate() {
            let SurfaceExactData::Cylinder {
                origin,
                x,
                y,
                axis,
                radius,
            } = surface.exact_data()
            else {
                continue;
            };
            let mut limits: Option<(Real, Real)> = None;
            for face in &self.data.faces {
                if face.surface.index() != index {
                    continue;
                }
                for wire in face.boundary_wires() {
                    for usage in &self.data.wires[wire.index()].edge_uses {
                        let pcurve =
                            &self.data.pcurves[self.data.edge_uses[usage.index()].pcurve.index()];
                        let bounds = pcurve.curve().bounds().map_err(GeometryError::from)?;
                        if let Some((lower, upper)) = &mut limits {
                            if decided_model_order(compare_reals(
                                bounds.min_x(),
                                lower,
                                crate::STRICT_PREDICATES,
                            ))? == std::cmp::Ordering::Less
                            {
                                *lower = bounds.min_x().clone();
                            }
                            if decided_model_order(compare_reals(
                                bounds.max_x(),
                                upper,
                                crate::STRICT_PREDICATES,
                            ))? == std::cmp::Ordering::Greater
                            {
                                *upper = bounds.max_x().clone();
                            }
                        } else {
                            limits = Some((bounds.min_x().clone(), bounds.max_x().clone()));
                        }
                    }
                }
            }
            let Some((lower, upper)) = limits else {
                return Err(GeometryError::UnsupportedTransform);
            };
            let profile = Curve3::circle_arc(origin, x, y, radius, lower, upper)?;
            // S(u,v) = origin + radius*(x*cos(u)+y*sin(u)) + axis*v
            // is unchanged on every complete trim image and has the same
            // oriented normal. No pcurve or edge correspondence is refitted.
            data.surfaces[index] = Surface::extrusion(profile, axis)?;
        }
        Ok(model)
    }
}
