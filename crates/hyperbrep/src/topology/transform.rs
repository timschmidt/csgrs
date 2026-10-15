//! Exact model transform operations.

use crate::topology::*;

impl Model {
    /// Applies one certified invertible affine transform to the entire model.
    ///
    /// Topology and typed IDs are retained. Exact curve and surface carriers
    /// are rebuilt in arena order, while pcurves and parameter maps remain in
    /// their authored surface coordinates.
    pub fn transformed(&self, transform: &Matrix4) -> Result<Self, GeometryError> {
        let reverses_orientation =
            affine_transform_orientation(transform)? == std::cmp::Ordering::Less;
        let vertices = self
            .data
            .vertices
            .iter()
            .map(|vertex| {
                transform
                    .transform_point3(&vertex.point)
                    .map(|point| Vertex { point })
                    .map_err(GeometryError::Numeric)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let curves = self
            .data
            .curves
            .iter()
            .map(|curve| curve.transformed(transform))
            .collect::<Result<Vec<_>, _>>()?;
        let surfaces = self
            .data
            .surfaces
            .iter()
            .map(|surface| surface.transformed(transform, reverses_orientation))
            .collect::<Result<Vec<_>, _>>()?;
        let pcurves = if reverses_orientation {
            let mut reflection_sums = vec![None::<Real>; self.data.pcurves.len()];
            for (edge_use_index, edge_use) in self.data.edge_uses.iter().enumerate() {
                let wire = self.data.edge_use_wire[edge_use_index];
                let face = self.data.wire_face[wire.index()];
                let surface = &self.data.surfaces[self.data.faces[face.index()].surface.index()];
                let reflection_sum = match surface.exact_data() {
                    SurfaceExactData::RationalBezier { .. } => Real::one(),
                    SurfaceExactData::Nurbs { u_knots, .. } => {
                        &u_knots[0] + &u_knots[u_knots.len() - 1]
                    }
                    _ => Real::zero(),
                };
                let slot = &mut reflection_sums[edge_use.pcurve.index()];
                if let Some(existing) = slot {
                    match compare_reals(existing, &reflection_sum, crate::STRICT_PREDICATES) {
                        PredicateOutcome::Decided {
                            value: std::cmp::Ordering::Equal,
                            ..
                        } => {}
                        PredicateOutcome::Decided { .. } => {
                            return Err(GeometryError::UnsupportedTransform);
                        }
                        PredicateOutcome::Unknown { needed, stage } => {
                            return Err(GeometryError::PredicateUnresolved { needed, stage });
                        }
                    }
                } else {
                    *slot = Some(reflection_sum);
                }
            }
            self.data
                .pcurves
                .iter()
                .zip(reflection_sums)
                .map(|(pcurve, reflection_sum)| {
                    pcurve.reflected_and_reversed_x(reflection_sum.unwrap_or_else(Real::zero))
                })
                .collect::<Result<Vec<_>, _>>()?
        } else {
            self.data.pcurves.clone()
        };
        let edge_uses = if reverses_orientation {
            self.data
                .edge_uses
                .iter()
                .map(|edge_use| {
                    let pcurve = &self.data.pcurves[edge_use.pcurve.index()];
                    Ok::<_, GeometryError>(EdgeUse {
                        edge: edge_use.edge,
                        direction: edge_use.direction.reversed(),
                        pcurve: edge_use.pcurve,
                        parameter_correspondence: edge_use
                            .parameter_correspondence
                            .reversed_pcurve(pcurve)?,
                        image_certificate: edge_use.image_certificate,
                    })
                })
                .collect::<Result<Vec<_>, _>>()?
        } else {
            self.data.edge_uses.clone()
        };
        let certified_cylinders = self
            .data
            .certified_cylinders
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedCylinderShell {
                            origin: transform
                                .transform_point3(&certificate.origin)
                                .map_err(GeometryError::Numeric)?,
                            axis: transform.transform_direction3(&certificate.axis),
                            radius: certificate.radius.clone(),
                            v_min: certificate.v_min.clone(),
                            v_max: certificate.v_max.clone(),
                            sphere_subtraction: certificate
                                .sphere_subtraction
                                .as_ref()
                                .map(|subtraction| {
                                    Ok::<_, GeometryError>(match subtraction {
                                        CertifiedCylinderSphereSubtraction::Void {
                                            center,
                                            radius,
                                        } => CertifiedCylinderSphereSubtraction::Void {
                                            center: transform
                                                .transform_point3(center)
                                                .map_err(GeometryError::Numeric)?,
                                            radius: radius.clone(),
                                        },
                                        CertifiedCylinderSphereSubtraction::Component {
                                            center,
                                            radius,
                                            side,
                                        } => CertifiedCylinderSphereSubtraction::Component {
                                            center: transform
                                                .transform_point3(center)
                                                .map_err(GeometryError::Numeric)?,
                                            radius: radius.clone(),
                                            side: *side,
                                        },
                                    })
                                })
                                .transpose()?,
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_spheres = self
            .data
            .certified_spheres
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedSphereShell {
                            center: transform
                                .transform_point3(&certificate.center)
                                .map_err(GeometryError::Numeric)?,
                            radius: certificate.radius.clone(),
                            region: match &certificate.region {
                                CertifiedSphereRegion::Whole => CertifiedSphereRegion::Whole,
                                CertifiedSphereRegion::Axial(clip) => {
                                    CertifiedSphereRegion::Axial(CertifiedSphereAxialClip {
                                        axis: transform.transform_direction3(&clip.axis),
                                        min: clip.min.clone(),
                                        max: clip.max.clone(),
                                    })
                                }
                                CertifiedSphereRegion::Radial(clip) => {
                                    CertifiedSphereRegion::Radial(CertifiedSphereRadialClip {
                                        axis: transform.transform_direction3(&clip.axis),
                                        radius: clip.radius.clone(),
                                        side: clip.side,
                                    })
                                }
                                CertifiedSphereRegion::FiniteCylinder(region) => {
                                    CertifiedSphereRegion::FiniteCylinder(
                                        CertifiedSphereFiniteCylinderRegion {
                                            origin: transform
                                                .transform_point3(&region.origin)
                                                .map_err(GeometryError::Numeric)?,
                                            axis: transform.transform_direction3(&region.axis),
                                            radius: region.radius.clone(),
                                            v_min: region.v_min.clone(),
                                            v_max: region.v_max.clone(),
                                            operation: region.operation,
                                        },
                                    )
                                }
                            },
                            voids: certificate
                                .voids
                                .iter()
                                .map(|void| {
                                    Ok(match void {
                                        CertifiedSphereVoid::Sphere { center, radius } => {
                                            CertifiedSphereVoid::Sphere {
                                                center: transform
                                                    .transform_point3(center)
                                                    .map_err(GeometryError::Numeric)?,
                                                radius: radius.clone(),
                                            }
                                        }
                                        CertifiedSphereVoid::Cylinder(cylinder) => {
                                            CertifiedSphereVoid::Cylinder(Box::new(
                                                CertifiedSphereCylinderVoid {
                                                    origin: transform
                                                        .transform_point3(&cylinder.origin)
                                                        .map_err(GeometryError::Numeric)?,
                                                    axis: transform
                                                        .transform_direction3(&cylinder.axis),
                                                    radius: cylinder.radius.clone(),
                                                    v_min: cylinder.v_min.clone(),
                                                    v_max: cylinder.v_max.clone(),
                                                },
                                            ))
                                        }
                                    })
                                })
                                .collect::<Result<Vec<_>, GeometryError>>()?,
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_sphere_pairs = self
            .data
            .certified_sphere_pairs
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedSpherePairShell {
                            first_center: transform
                                .transform_point3(&certificate.first_center)
                                .map_err(GeometryError::Numeric)?,
                            first_radius: certificate.first_radius.clone(),
                            second_center: transform
                                .transform_point3(&certificate.second_center)
                                .map_err(GeometryError::Numeric)?,
                            second_radius: certificate.second_radius.clone(),
                            kind: certificate.kind,
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_tori = self
            .data
            .certified_tori
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedTorusShell {
                            center: transform
                                .transform_point3(&certificate.center)
                                .map_err(GeometryError::Numeric)?,
                            axis: transform.transform_direction3(&certificate.axis),
                            major_radius: certificate.major_radius.clone(),
                            minor_radius: certificate.minor_radius.clone(),
                            region: match &certificate.region {
                                CertifiedTorusRegion::Whole => CertifiedTorusRegion::Whole,
                                CertifiedTorusRegion::Axial { min, max } => {
                                    CertifiedTorusRegion::Axial {
                                        min: min.clone(),
                                        max: max.clone(),
                                    }
                                }
                                CertifiedTorusRegion::LongitudinalHalf { interior_normal } => {
                                    CertifiedTorusRegion::LongitudinalHalf {
                                        interior_normal: transform
                                            .transform_direction3(interior_normal),
                                    }
                                }
                            },
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_revolutions = self
            .data
            .certified_revolutions
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedRevolutionShell {
                            axis_origin: transform
                                .transform_point3(&certificate.axis_origin)
                                .map_err(GeometryError::Numeric)?,
                            axis: transform.transform_direction3(&certificate.axis),
                            profile: certificate.profile.clone(),
                            voids: certificate.voids.clone(),
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_lofts = self
            .data
            .certified_lofts
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedLoftShell {
                            origin: transform
                                .transform_point3(&certificate.origin)
                                .map_err(GeometryError::Numeric)?,
                            u: transform.transform_direction3(&certificate.u),
                            v: transform.transform_direction3(&certificate.v),
                            height_axis: transform.transform_direction3(&certificate.height_axis),
                            spans: certificate.spans.clone(),
                            parameter_volume: certificate.parameter_volume.clone(),
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_curve_sweeps = self
            .data
            .certified_curve_sweeps
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedCurveSweepShell {
                            profile: certificate.profile.clone(),
                            holes: certificate.holes.clone(),
                            path: certificate.path.transformed(transform)?,
                            u_path: transform_vector_curve(&certificate.u_path, transform)?,
                            v_path: transform_vector_curve(&certificate.v_path, transform)?,
                            area_scale_integral: certificate.area_scale_integral.clone(),
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_cone_frustums = self
            .data
            .certified_cone_frustums
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedConeFrustumShell {
                            apex: transform
                                .transform_point3(&certificate.apex)
                                .map_err(GeometryError::Numeric)?,
                            axis: transform.transform_direction3(&certificate.axis),
                            semi_angle: certificate.semi_angle.clone(),
                            v_min: certificate.v_min.clone(),
                            v_max: certificate.v_max.clone(),
                            region: match &certificate.region {
                                CertifiedConeFrustumRegion::Whole => {
                                    CertifiedConeFrustumRegion::Whole
                                }
                                CertifiedConeFrustumRegion::LongitudinalHalf {
                                    interior_normal,
                                } => CertifiedConeFrustumRegion::LongitudinalHalf {
                                    interior_normal: transform
                                        .transform_direction3(interior_normal),
                                },
                            },
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_prisms = self
            .data
            .certified_prisms
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedPrismShell {
                            profile: certificate.profile.clone(),
                            origin: transform
                                .transform_point3(&certificate.origin)
                                .map_err(GeometryError::Numeric)?,
                            u: transform.transform_direction3(&certificate.u),
                            v: transform.transform_direction3(&certificate.v),
                            extrusion: transform.transform_direction3(&certificate.extrusion),
                            parameter_min: certificate.parameter_min.clone(),
                            parameter_max: certificate.parameter_max.clone(),
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let wires = if reverses_orientation {
            self.data
                .wires
                .iter()
                .map(|wire| {
                    let mut edge_uses = wire.edge_uses.clone();
                    edge_uses.reverse();
                    Wire {
                        edge_uses,
                        simplicity_certificate: wire.simplicity_certificate,
                    }
                })
                .collect()
        } else {
            self.data.wires.clone()
        };
        Ok(Self {
            data: Arc::new(ModelData {
                vertices,
                curves,
                pcurves,
                surfaces,
                edges: self.data.edges.clone(),
                edge_uses,
                wires,
                faces: self.data.faces.clone(),
                shells: self.data.shells.clone(),
                solids: self.data.solids.clone(),
                vertex_edges: self.data.vertex_edges.clone(),
                edge_uses_by_edge: self.data.edge_uses_by_edge.clone(),
                edge_use_wire: self.data.edge_use_wire.clone(),
                wire_face: self.data.wire_face.clone(),
                face_shell: self.data.face_shell.clone(),
                shell_solid: self.data.shell_solid.clone(),
                solid_certificates_valid: self.data.solid_certificates_valid.clone(),
                certified_cylinders,
                certified_spheres,
                certified_sphere_pairs,
                certified_cone_frustums,
                certified_tori,
                certified_revolutions,
                certified_lofts,
                certified_curve_sweeps,
                certified_prisms,
                bounds: OnceLock::new(),
                face_contours: (0..self.data.faces.len())
                    .map(|_| OnceLock::new())
                    .collect(),
            }),
        })
    }
}
