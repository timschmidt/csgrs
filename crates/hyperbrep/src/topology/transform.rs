//! Exact model transform operations.

use crate::geometry::TransformMetric;
use crate::topology::*;

#[cfg(test)]
mod tests;

impl ArenaModel {
    /// Each pcurve is transported according to its owning surface chart.
    /// A single authored pcurve can be shared by different chart families;
    /// such uses must no longer alias when their coordinate maps differ.
    fn transformed_axial_charts(
        &self,
        scale: &Real,
        reflected: bool,
    ) -> Result<(Vec<Pcurve>, Vec<EdgeUse>), GeometryError> {
        let mut pcurves = self.data.pcurves.clone();
        let mut maps: Vec<Vec<(bool, Real, PcurveId)>> = vec![Vec::new(); pcurves.len()];
        let mut uses = Vec::with_capacity(self.data.edge_uses.len());
        for (index, edge_use) in self.data.edge_uses.iter().enumerate() {
            let face = self.data.wire_face[self.data.edge_use_wire[index].index()];
            let surface = &self.data.surfaces[self.data.faces[face.index()].surface.index()];
            let axial = matches!(surface.kind(), SurfaceKind::Cylinder | SurfaceKind::Cone);
            let sum = if reflected {
                surface.reflection_sum()
            } else {
                Real::zero()
            };
            let source = &self.data.pcurves[edge_use.pcurve.index()];
            let variants = &mut maps[edge_use.pcurve.index()];
            let mut matching = None;
            for (prior_axial, prior_sum, id) in variants.iter() {
                if *prior_axial == axial && prior_sum == &sum {
                    matching = Some(*id);
                    break;
                }
            }
            let pcurve = if let Some(id) = matching {
                id
            } else {
                let value = if axial {
                    source.scaled_axial(scale)?
                } else {
                    source.clone()
                };
                let value = if reflected {
                    value.reflected_and_reversed_x(sum.clone())?
                } else {
                    value
                };
                let id = if variants.is_empty() {
                    pcurves[edge_use.pcurve.index()] = value;
                    edge_use.pcurve
                } else {
                    let id = PcurveId::from_index(pcurves.len())
                        .ok_or(GeometryError::Numeric(hyperreal::Problem::Exhausted))?;
                    pcurves
                        .try_reserve(1)
                        .map_err(|_| GeometryError::Numeric(hyperreal::Problem::Exhausted))?;
                    pcurves.push(value);
                    id
                };
                variants.push((axial, sum, id));
                id
            };
            uses.push(EdgeUse {
                edge: edge_use.edge,
                direction: if reflected {
                    edge_use.direction.reversed()
                } else {
                    edge_use.direction
                },
                pcurve,
                parameter_correspondence: if reflected {
                    edge_use.parameter_correspondence.reversed_pcurve(source)?
                } else {
                    edge_use.parameter_correspondence.clone()
                },
                image_certificate: edge_use.image_certificate,
            });
        }
        Ok((pcurves, uses))
    }

    /// Applies one certified invertible affine transform to the entire model.
    ///
    /// Exact curve and surface carriers are rebuilt in arena order. Analytic
    /// lengths and axial surface charts follow a certified similarity scale.
    /// Shared pcurves acquire separate records if owning face charts differ;
    /// edge parameters, topology and source selections retain their meaning.
    pub fn transformed(&self, transform: &Matrix4) -> Result<Self, GeometryError> {
        crate::context::strict(|| {
            let reverses_orientation =
                affine_transform_orientation(transform)? == std::cmp::Ordering::Less;
            let metric = TransformMetric::new(transform);
            let curves = self
                .data
                .curves
                .iter()
                .map(|curve| curve.transformed_with_metric(&metric))
                .collect::<Result<Vec<_>, _>>()?;
            let curve_sources = std::cell::OnceCell::new();
            let vertices = self
                .data
                .vertices
                .iter()
                .map(|vertex| {
                    let point = if let Some(coordinates) = vertex.point.try_coordinates() {
                        transform
                            .transform_point3(coordinates)
                            .map_err(GeometryError::Numeric)?
                            .into()
                    } else if let Some((source, parameter)) = vertex.point.curve_source()
                        && let Some(index) = curve_sources
                            .get_or_init(|| {
                                self.data
                                    .curves
                                    .iter()
                                    .enumerate()
                                    .map(|(index, curve)| (curve.identity_address(), index))
                                    .collect::<HashMap<_, _>>()
                            })
                            .get(&source.identity_address())
                    {
                        curves[*index].point_at(parameter)?
                    } else {
                        vertex.point.transformed(transform)?
                    };
                    Ok(Vertex { point })
                })
                .collect::<Result<Vec<_>, GeometryError>>()?;
            let surfaces = self
                .data
                .surfaces
                .iter()
                .map(|surface| surface.transformed(&metric, reverses_orientation))
                .collect::<Result<Vec<_>, _>>()?;
            let axial_scale =
                if self.data.surfaces.iter().any(|surface| {
                    matches!(surface.kind(), SurfaceKind::Cylinder | SurfaceKind::Cone)
                }) {
                    Some(metric.scale()?)
                } else {
                    None
                };
            let (pcurves, edge_uses) = if let Some(scale) =
                axial_scale.filter(|scale| *scale != &Real::one())
            {
                self.transformed_axial_charts(scale, reverses_orientation)?
            } else {
                let pcurves = if reverses_orientation {
                    let mut reflection_sums = vec![None::<Real>; self.data.pcurves.len()];
                    for (edge_use_index, edge_use) in self.data.edge_uses.iter().enumerate() {
                        let wire = self.data.edge_use_wire[edge_use_index];
                        let face = self.data.wire_face[wire.index()];
                        let surface =
                            &self.data.surfaces[self.data.faces[face.index()].surface.index()];
                        let reflection_sum = surface.reflection_sum();
                        let slot = &mut reflection_sums[edge_use.pcurve.index()];
                        if let Some(existing) = slot {
                            match compare_reals(existing, &reflection_sum, crate::STRICT_PREDICATES)
                            {
                                PredicateOutcome::Decided {
                                    value: std::cmp::Ordering::Equal,
                                    ..
                                } => {}
                                PredicateOutcome::Decided { .. } => {
                                    return Err(GeometryError::UnsupportedTransform);
                                }
                                PredicateOutcome::Unknown { needed, stage } => {
                                    return Err(GeometryError::PredicateUnresolved {
                                        needed,
                                        stage,
                                    });
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
                            pcurve
                                .reflected_and_reversed_x(reflection_sum.unwrap_or_else(Real::zero))
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
                (pcurves, edge_uses)
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
                                axis: metric.unit_direction(&certificate.axis)?,
                                radius: metric.length(&certificate.radius)?,
                                v_min: metric.length(&certificate.v_min)?,
                                v_max: metric.length(&certificate.v_max)?,
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
                                                radius: metric.length(radius)?,
                                            },
                                            CertifiedCylinderSphereSubtraction::Component {
                                                center,
                                                radius,
                                                side,
                                            } => CertifiedCylinderSphereSubtraction::Component {
                                                center: transform
                                                    .transform_point3(center)
                                                    .map_err(GeometryError::Numeric)?,
                                                radius: metric.length(radius)?,
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
                                radius: metric.length(&certificate.radius)?,
                                region: match &certificate.region {
                                    CertifiedSphereRegion::Whole => CertifiedSphereRegion::Whole,
                                    CertifiedSphereRegion::Axial(clip) => {
                                        CertifiedSphereRegion::Axial(CertifiedSphereAxialClip {
                                            axis: metric.unit_direction(&clip.axis)?,
                                            min: metric.length(&clip.min)?,
                                            max: metric.length(&clip.max)?,
                                        })
                                    }
                                    CertifiedSphereRegion::Radial(clip) => {
                                        CertifiedSphereRegion::Radial(CertifiedSphereRadialClip {
                                            axis: metric.unit_direction(&clip.axis)?,
                                            radius: metric.length(&clip.radius)?,
                                            side: clip.side,
                                        })
                                    }
                                    CertifiedSphereRegion::FiniteCylinder(region) => {
                                        CertifiedSphereRegion::FiniteCylinder(
                                            CertifiedSphereFiniteCylinderRegion {
                                                origin: transform
                                                    .transform_point3(&region.origin)
                                                    .map_err(GeometryError::Numeric)?,
                                                axis: metric.unit_direction(&region.axis)?,
                                                radius: metric.length(&region.radius)?,
                                                v_min: metric.length(&region.v_min)?,
                                                v_max: metric.length(&region.v_max)?,
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
                                                    radius: metric.length(radius)?,
                                                }
                                            }
                                            CertifiedSphereVoid::Cylinder(cylinder) => {
                                                CertifiedSphereVoid::Cylinder(Box::new(
                                                    CertifiedSphereCylinderVoid {
                                                        origin: transform
                                                            .transform_point3(&cylinder.origin)
                                                            .map_err(GeometryError::Numeric)?,
                                                        axis: metric
                                                            .unit_direction(&cylinder.axis)?,
                                                        radius: metric.length(&cylinder.radius)?,
                                                        v_min: metric.length(&cylinder.v_min)?,
                                                        v_max: metric.length(&cylinder.v_max)?,
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
                                first_radius: metric.length(&certificate.first_radius)?,
                                second_center: transform
                                    .transform_point3(&certificate.second_center)
                                    .map_err(GeometryError::Numeric)?,
                                second_radius: metric.length(&certificate.second_radius)?,
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
                                axis: metric.unit_direction(&certificate.axis)?,
                                major_radius: metric.length(&certificate.major_radius)?,
                                minor_radius: metric.length(&certificate.minor_radius)?,
                                region: match &certificate.region {
                                    CertifiedTorusRegion::Whole => CertifiedTorusRegion::Whole,
                                    CertifiedTorusRegion::Axial { min, max } => {
                                        CertifiedTorusRegion::Axial {
                                            min: metric.length(min)?,
                                            max: metric.length(max)?,
                                        }
                                    }
                                    CertifiedTorusRegion::LongitudinalHalf { interior_normal } => {
                                        CertifiedTorusRegion::LongitudinalHalf {
                                            interior_normal: metric
                                                .unit_direction(interior_normal)?,
                                        }
                                    }
                                    CertifiedTorusRegion::SectorBand { angular, min, max } => {
                                        CertifiedTorusRegion::SectorBand {
                                            angular: CertifiedAngularSector {
                                                interior_normals: angular
                                                    .interior_normals
                                                    .iter()
                                                    .map(|normal| metric.unit_direction(normal))
                                                    .collect::<Result<Vec<_>, GeometryError>>()?,
                                                sweep: angular.sweep.clone(),
                                            },
                                            min: metric.length(min)?,
                                            max: metric.length(max)?,
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
                                axis: metric.unit_direction(&certificate.axis)?,
                                profile: scale_revolution_boundary(
                                    &certificate.profile,
                                    metric.scale()?,
                                )?,
                                voids: certificate
                                    .voids
                                    .iter()
                                    .map(|boundary| {
                                        scale_revolution_boundary(boundary, metric.scale()?)
                                    })
                                    .collect::<Result<Vec<_>, GeometryError>>()?,
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
                                height_axis: transform
                                    .transform_direction3(&certificate.height_axis),
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
                                axis: metric.unit_direction(&certificate.axis)?,
                                semi_angle: certificate.semi_angle.clone(),
                                v_min: metric.length(&certificate.v_min)?,
                                v_max: metric.length(&certificate.v_max)?,
                                region: match &certificate.region {
                                    CertifiedConeFrustumRegion::Whole => {
                                        CertifiedConeFrustumRegion::Whole
                                    }
                                    CertifiedConeFrustumRegion::LongitudinalHalf {
                                        interior_normal,
                                    } => CertifiedConeFrustumRegion::LongitudinalHalf {
                                        interior_normal: metric.unit_direction(interior_normal)?,
                                    },
                                    CertifiedConeFrustumRegion::Sector(angular) => {
                                        CertifiedConeFrustumRegion::Sector(CertifiedAngularSector {
                                            interior_normals: angular
                                                .interior_normals
                                                .iter()
                                                .map(|normal| metric.unit_direction(normal))
                                                .collect::<Result<Vec<_>, GeometryError>>()?,
                                            sweep: angular.sweep.clone(),
                                        })
                                    }
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
        })
    }
}

fn scale_revolution_boundary(
    boundary: &CertifiedRevolutionBoundary,
    scale: &Real,
) -> Result<CertifiedRevolutionBoundary, GeometryError> {
    if scale == &Real::one() {
        return Ok(boundary.clone());
    }
    let map = hypercurve::Similarity2::try_from_real_affine(
        scale.clone(),
        Real::zero(),
        Real::zero(),
        scale.clone(),
        Real::zero(),
        Real::zero(),
    )?;
    Ok(match boundary {
        CertifiedRevolutionBoundary::Native(contour) => {
            CertifiedRevolutionBoundary::Native(contour.transform_similarity(&map)?)
        }
        CertifiedRevolutionBoundary::Curved(path) => {
            CertifiedRevolutionBoundary::Curved(path.transform_similarity(&map)?)
        }
    })
}
