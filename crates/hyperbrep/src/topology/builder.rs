//! Validated construction: builder.

use crate::topology::*;

impl ArenaModelBuilder {
    /// Constructs an empty model builder.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds an exact model-space vertex.
    pub fn vertex(&mut self, point: Point3) -> Result<VertexId, BuildError> {
        crate::context::strict(|| {
            let id = VertexId::from_index(self.vertices.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
            self.vertices.push(Vertex { point });
            self.vertex_edges.push(Vec::new());
            Ok(id)
        })
    }

    /// Adds an immutable spatial curve.
    pub fn curve(&mut self, curve: Curve3) -> Result<Curve3Id, BuildError> {
        crate::context::strict(|| {
            let id = Curve3Id::from_index(self.curves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
            self.curves.push(curve);
            Ok(id)
        })
    }

    /// Adds an immutable face-local pcurve.
    pub fn pcurve(&mut self, pcurve: Pcurve) -> Result<PcurveId, BuildError> {
        crate::context::strict(|| {
            let id = PcurveId::from_index(self.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            self.pcurves.push(pcurve);
            Ok(id)
        })
    }

    /// Adds an immutable parameterized surface.
    pub fn surface(&mut self, surface: Surface) -> Result<SurfaceId, BuildError> {
        crate::context::strict(|| {
            let id = SurfaceId::from_index(self.surfaces.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Surface))?;
            self.surfaces.push(surface);
            Ok(id)
        })
    }

    /// Adds a topological edge with exact geometry and a retained subdomain.
    pub fn edge(
        &mut self,
        start: VertexId,
        end: VertexId,
        curve: Curve3Id,
        domain: ParameterDomain,
    ) -> Result<EdgeId, BuildError> {
        crate::context::strict(|| {
            self.edge_with_endpoint_certificate(start, end, curve, domain, false)
        })
    }

    pub(crate) fn edge_with_endpoint_certificate(
        &mut self,
        start: VertexId,
        end: VertexId,
        curve: Curve3Id,
        domain: ParameterDomain,
        endpoint_certificate: bool,
    ) -> Result<EdgeId, BuildError> {
        if start == end {
            return Err(BuildError::DegenerateEdge);
        }
        let start_point = self.vertex_ref(start)?.point();
        let end_point = self.vertex_ref(end)?.point();
        let curve_ref = self.curve_ref(curve)?;
        if !endpoint_certificate {
            require_curve_point_at_parameter(
                curve_ref,
                domain.start(),
                start_point,
                BuildError::EdgeEndpointMismatch {
                    endpoint: Endpoint::Start,
                },
            )?;
            require_curve_point_at_parameter(
                curve_ref,
                domain.end(),
                end_point,
                BuildError::EdgeEndpointMismatch {
                    endpoint: Endpoint::End,
                },
            )?;
        }

        let id = EdgeId::from_index(self.edges.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
        self.edges.push(Edge {
            start,
            end,
            curve,
            domain,
            endpoint_certificate,
        });
        self.vertex_edges[start.index()].push(id);
        self.vertex_edges[end.index()].push(id);
        self.edge_uses_by_edge.push(Vec::new());
        Ok(id)
    }

    /// Adds an oriented edge use with face-local pcurve geometry.
    pub fn edge_use(
        &mut self,
        edge: EdgeId,
        direction: Direction,
        pcurve: PcurveId,
        parameter_correspondence: ParameterCorrespondence,
    ) -> Result<EdgeUseId, BuildError> {
        crate::context::strict(|| {
            self.edge_use_with_image_certificate(
                edge,
                direction,
                pcurve,
                parameter_correspondence,
                false,
            )
        })
    }

    pub(crate) fn edge_use_with_image_certificate(
        &mut self,
        edge: EdgeId,
        direction: Direction,
        pcurve: PcurveId,
        parameter_correspondence: ParameterCorrespondence,
        image_certificate: bool,
    ) -> Result<EdgeUseId, BuildError> {
        self.edge_ref(edge)?;
        self.pcurve_ref(pcurve)?;
        let id = EdgeUseId::from_index(self.edge_uses.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
        self.edge_uses.push(EdgeUse {
            edge,
            direction,
            pcurve,
            parameter_correspondence,
            image_certificate,
        });
        self.edge_uses_by_edge[edge.index()].push(id);
        self.edge_use_wire.push(None);
        Ok(id)
    }

    /// Adds an ordered, connected, closed wire.
    pub fn wire(&mut self, edge_uses: Vec<EdgeUseId>) -> Result<WireId, BuildError> {
        crate::context::strict(|| self.wire_with_simplicity_certificate(edge_uses, false))
    }

    pub(crate) fn wire_with_simplicity_certificate(
        &mut self,
        edge_uses: Vec<EdgeUseId>,
        simplicity_certificate: bool,
    ) -> Result<WireId, BuildError> {
        if edge_uses.is_empty() {
            return Err(BuildError::EmptyWire);
        }
        let mut unique = HashSet::with_capacity(edge_uses.len());
        for edge_use in &edge_uses {
            self.edge_use_ref(*edge_use)?;
            if !unique.insert(*edge_use) {
                return Err(BuildError::DuplicateEdgeUse(*edge_use));
            }
            if self.edge_use_wire[edge_use.index()].is_some() {
                return Err(BuildError::EdgeUseAlreadyOwned(*edge_use));
            }
        }
        for (index, pair) in edge_uses.windows(2).enumerate() {
            let (_, first_end) = self.directed_vertices(pair[0])?;
            let (second_start, _) = self.directed_vertices(pair[1])?;
            if first_end != second_start {
                return Err(BuildError::DisconnectedWire { at: index });
            }
        }
        let (first_start, _) = self.directed_vertices(edge_uses[0])?;
        let (_, last_end) = self.directed_vertices(*edge_uses.last().expect("nonempty wire"))?;
        if first_start != last_end {
            return Err(BuildError::OpenWire);
        }

        if simplicity_certificate {
            let line_pcurves = edge_uses
                .iter()
                .map(|edge_use| {
                    let pcurve = self.edge_use_ref(*edge_use)?.pcurve;
                    let line = self.pcurve_ref(pcurve)?.line_segment();
                    Ok(line.map(|line| (pcurve, line.start().clone(), line.end().clone())))
                })
                .collect::<Result<Option<Vec<_>>, BuildError>>()?;
            if let Some(line_pcurves) = line_pcurves {
                let uniquely_owned = line_pcurves.iter().all(|(pcurve, _, _)| {
                    self.edge_uses
                        .iter()
                        .filter(|edge_use| edge_use.pcurve == *pcurve)
                        .count()
                        == 1
                });
                if uniquely_owned {
                    for (index, (pcurve, start, original_end)) in line_pcurves.iter().enumerate() {
                        let end = line_pcurves
                            .get(index + 1)
                            .map_or(original_end, |(_, start, _)| start);
                        self.pcurves[pcurve.index()] = Pcurve::new(Curve2::from(
                            LineSeg2::try_new(start.clone(), end.clone())
                                .map_err(GeometryError::from)?,
                        ));
                    }
                }
            }
        }

        let id = WireId::from_index(self.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        self.wires.push(Wire {
            edge_uses: edge_uses.clone(),
            simplicity_certificate,
        });
        self.wire_face.push(None);
        for edge_use in edge_uses {
            self.edge_use_wire[edge_use.index()] = Some(id);
        }
        Ok(id)
    }

    /// Adds a face and certifies edge/pcurve/surface agreement.
    pub fn face(
        &mut self,
        surface: SurfaceId,
        orientation: Orientation,
        outer: WireId,
        inner: Vec<WireId>,
    ) -> Result<FaceId, BuildError> {
        crate::context::strict(|| {
            let spherical = self.surface_ref(surface)?.kind() == SurfaceKind::Sphere;
            self.wire_ref(outer)?;
            let mut wires = Vec::with_capacity(inner.len() + 1);
            wires.push(outer);
            wires.extend(inner.iter().copied());
            let mut unique = HashSet::with_capacity(wires.len());
            for wire in &wires {
                self.wire_ref(*wire)?;
                if !unique.insert(*wire) {
                    return Err(BuildError::DuplicateWire(*wire));
                }
                if self.wire_face[wire.index()].is_some() {
                    return Err(BuildError::WireAlreadyOwned(*wire));
                }
                self.validate_wire_image(*wire, surface)?;
            }
            if spherical {
                if inner.len() > 1 {
                    return Err(BuildError::UnsupportedSphericalTrim(outer));
                }
                if let Some(upper_wire) = inner.first() {
                    self.validate_spherical_trim(outer, orientation)?;
                    self.validate_spherical_trim(*upper_wire, orientation)?;
                    let (lower_latitude, lower_direction) =
                        self.spherical_trim_coordinates(outer)?;
                    let (upper_latitude, upper_direction) =
                        self.spherical_trim_coordinates(*upper_wire)?;
                    let expected_lower = match orientation {
                        Orientation::Forward => std::cmp::Ordering::Greater,
                        Orientation::Reversed => std::cmp::Ordering::Less,
                    };
                    if decided_model_order(compare_reals(
                        &lower_latitude,
                        &upper_latitude,
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Less
                        || lower_direction != expected_lower
                        || upper_direction == expected_lower
                    {
                        return Err(BuildError::InvalidSphericalTrim(outer));
                    }
                } else {
                    match self.validate_spherical_trim(outer, orientation) {
                        Ok(()) => {}
                        Err(BuildError::UnsupportedSphericalTrim(_)) => {
                            self.validate_wire_simplicity(outer)?;
                            self.validate_wire_orientation(outer, orientation, true, surface)?;
                        }
                        Err(error) => return Err(error),
                    }
                }
            } else {
                for wire in &wires {
                    if !self.wire_ref(*wire)?.simplicity_certificate {
                        self.validate_wire_simplicity(*wire)?;
                    }
                }
                let outer_certified = self.wire_ref(outer)?.simplicity_certificate;
                if !outer_certified {
                    self.validate_wire_orientation(outer, orientation, true, surface)?;
                }
                for wire in &inner {
                    if !self.wire_ref(*wire)?.simplicity_certificate {
                        self.validate_wire_orientation(*wire, orientation, false, surface)?;
                    }
                }
                if !outer_certified {
                    self.validate_wire_nesting(outer, &inner)?;
                }
            }

            let id = FaceId::from_index(self.faces.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
            self.faces.push(Face {
                surface,
                orientation,
                boundary: FaceBoundary::Trimmed { outer, inner },
            });
            self.face_shell.push(None);
            for wire in wires {
                self.wire_face[wire.index()] = Some(id);
            }
            Ok(id)
        })
    }

    /// Adds the complete closed support surface as one boundaryless face.
    ///
    /// This is currently defined for spheres. It represents the closed
    /// geometric carrier directly instead of inventing seam edges or
    /// zero-length pole topology.
    pub fn whole_face(
        &mut self,
        surface: SurfaceId,
        orientation: Orientation,
    ) -> Result<FaceId, BuildError> {
        crate::context::strict(|| {
            let support = self.surface_ref(surface)?;
            if support.kind() != SurfaceKind::Sphere {
                return Err(BuildError::UnsupportedWholeSurface(support.kind()));
            }
            let id = FaceId::from_index(self.faces.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
            self.faces.push(Face {
                surface,
                orientation,
                boundary: FaceBoundary::WholeSurface,
            });
            self.face_shell.push(None);
            Ok(id)
        })
    }

    /// Adds one edge-connected shell.
    pub fn shell(&mut self, faces: Vec<FaceId>) -> Result<ShellId, BuildError> {
        crate::context::strict(|| {
            if faces.is_empty() {
                return Err(BuildError::EmptyShell);
            }
            let mut unique = HashSet::with_capacity(faces.len());
            for face in &faces {
                self.face_ref(*face)?;
                if !unique.insert(*face) {
                    return Err(BuildError::DuplicateFace(*face));
                }
                if self.face_shell[face.index()].is_some() {
                    return Err(BuildError::FaceAlreadyOwned(*face));
                }
            }
            if !self.faces_connected(&faces)? {
                return Err(BuildError::DisconnectedShell);
            }

            let id = ShellId::from_index(self.shells.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Shell))?;
            self.shells.push(Shell {
                faces: faces.clone(),
            });
            self.shell_solid.push(None);
            for face in faces {
                self.face_shell[face.index()] = Some(id);
            }
            Ok(id)
        })
    }

    /// Adds a regular solid bounded by one certified closed shell.
    pub fn solid(&mut self, outer: ShellId, voids: Vec<ShellId>) -> Result<SolidId, BuildError> {
        crate::context::strict(|| {
            self.shell_ref(outer)?;
            let mut shells = Vec::with_capacity(voids.len() + 1);
            shells.push(outer);
            shells.extend(voids.iter().copied());
            let mut unique = HashSet::with_capacity(shells.len());
            for shell in &shells {
                self.shell_ref(*shell)?;
                if !unique.insert(*shell) {
                    return Err(BuildError::DuplicateShell(*shell));
                }
                if self.shell_solid[shell.index()].is_some() {
                    return Err(BuildError::ShellAlreadyOwned(*shell));
                }
                self.validate_closed_shell(*shell)?;
            }
            let candidate = Solid {
                outer,
                voids: voids.clone(),
            };
            let sphere = self.certified_sphere_solid(&candidate)?;
            let sphere_pair = self.certified_sphere_pair_shell(outer)?;
            let revolution = self.certified_revolution_shell(outer)?;
            let cylinder = self.certified_cylinder_solid(&candidate)?;
            let cone_frustum = self.certified_cone_frustum_shell(outer)?;
            let torus = self.certified_torus_shell(outer)?;
            let analytic_closed = sphere.is_some()
                || sphere_pair.is_some()
                || revolution.is_some()
                || cylinder.is_some()
                || cone_frustum.is_some()
                || torus.is_some();
            let simple_prism = if analytic_closed {
                false
            } else {
                self.certify_simple_prism_shell(outer)?
                    || self.certify_internally_partitioned_prism_shell(outer)?
            };
            let line_arc_prism = if analytic_closed || simple_prism {
                false
            } else {
                self.certify_extrusion_prism_shell(outer)?
            };
            let curve_sweep = if analytic_closed || simple_prism || line_arc_prism {
                None
            } else {
                self.certified_curve_sweep_shell(outer)?
            };
            let loft = if analytic_closed || simple_prism || line_arc_prism || curve_sweep.is_some()
            {
                None
            } else {
                self.certified_loft_shell(outer)?
            };
            let convex_planar = if analytic_closed
                || simple_prism
                || line_arc_prism
                || curve_sweep.is_some()
                || loft.is_some()
            {
                false
            } else {
                self.certify_convex_planar_shell(outer)?
            };
            let general_planar = if analytic_closed
                || simple_prism
                || line_arc_prism
                || curve_sweep.is_some()
                || loft.is_some()
                || convex_planar
            {
                false
            } else {
                self.certify_planar_shell_non_self_intersection(outer)?
            };
            if !analytic_closed {
                self.validate_outer_shell_orientation(outer)?;
            }
            if !analytic_closed
                && !simple_prism
                && !line_arc_prism
                && cone_frustum.is_none()
                && torus.is_none()
                && curve_sweep.is_none()
                && loft.is_none()
                && !convex_planar
                && !general_planar
            {
                return Err(BuildError::UnsupportedSolidShell(outer));
            }
            for void_shell in &voids {
                let sphere_void = self
                    .certified_oriented_sphere_shell(*void_shell, Orientation::Reversed)?
                    .is_some();
                let revolution_void = self
                    .certified_oriented_revolution_shell(*void_shell, Orientation::Reversed)?
                    .is_some();
                let cylinder_void = self
                    .certified_oriented_cylinder_shell(*void_shell, Orientation::Reversed)?
                    .is_some();
                if !sphere_void && !revolution_void && !cylinder_void {
                    self.validate_void_shell_orientation(*void_shell)?;
                }
                let simple_prism_void = if sphere_void || revolution_void || cylinder_void {
                    false
                } else {
                    self.certify_simple_prism_shell(*void_shell)?
                };
                let general_planar_void =
                    if sphere_void || revolution_void || cylinder_void || simple_prism_void {
                        false
                    } else {
                        self.certify_planar_shell_non_self_intersection(*void_shell)?
                    };
                if !sphere_void
                    && !revolution_void
                    && !cylinder_void
                    && !simple_prism_void
                    && !general_planar_void
                {
                    return Err(BuildError::UnsupportedSolidShell(*void_shell));
                }
            }
            self.validate_void_shell_nesting(outer, &voids)?;

            let id = SolidId::from_index(self.solids.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Solid))?;
            self.solids.push(Solid {
                outer,
                voids: voids.clone(),
            });
            self.retained_solid_certificates.push(None);
            for shell in shells {
                self.shell_solid[shell.index()] = Some(id);
            }
            Ok(id)
        })
    }

    pub(in crate::topology) fn solid_with_retained_certificate(
        &mut self,
        outer: ShellId,
        voids: Vec<ShellId>,
        certificate: CertifiedSolid,
    ) -> Result<SolidId, BuildError> {
        self.shell_ref(outer)?;
        let mut shells = Vec::with_capacity(voids.len() + 1);
        shells.push(outer);
        shells.extend(voids.iter().copied());
        let mut unique = HashSet::with_capacity(shells.len());
        for shell in &shells {
            self.shell_ref(*shell)?;
            if !unique.insert(*shell) {
                return Err(BuildError::DuplicateShell(*shell));
            }
            if self.shell_solid[shell.index()].is_some() {
                return Err(BuildError::ShellAlreadyOwned(*shell));
            }
            self.validate_closed_shell(*shell)?;
        }

        let id = SolidId::from_index(self.solids.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Solid))?;
        self.solids.push(Solid {
            outer,
            voids: voids.clone(),
        });
        self.retained_solid_certificates.push(Some(certificate));
        for shell in shells {
            self.shell_solid[shell.index()] = Some(id);
        }
        Ok(id)
    }

    /// Adds a solid whose topology was already certified by an authoritative
    /// curve-region construction, without asking shape recognizers to rebuild
    /// that boundary as an independent path during commit.
    pub(crate) fn solid_with_curve_region_prism_certificate(
        &mut self,
        outer: ShellId,
        region: CurveRegion2,
        area: Real,
        z_min: Real,
        z_max: Real,
    ) -> Result<SolidId, BuildError> {
        self.solid_with_retained_certificate(
            outer,
            Vec::new(),
            CertifiedSolid {
                cylinder: None,
                sphere: None,
                sphere_pair: None,
                cone_frustum: None,
                torus: None,
                revolution: None,
                loft: None,
                curve_sweep: None,
                prism: Some(CertifiedPrismShell {
                    profile: CertifiedPrismProfile::CurveRegion { region, area },
                    origin: Point3::new(Real::zero(), Real::zero(), z_min.clone()),
                    u: Vector3::x(),
                    v: Vector3::y(),
                    extrusion: Vector3::from_xyz(Real::zero(), Real::zero(), z_max - z_min),
                    parameter_min: Real::zero(),
                    parameter_max: Real::one(),
                }),
            },
        )
    }

    /// Validates global ownership and commits an immutable model.
    pub fn finish(mut self) -> Result<ArenaModel, ValidationReport> {
        crate::context::strict(|| {
            let mut errors = Vec::new();
            for (index, owner) in self.edge_use_wire.iter().enumerate() {
                if owner.is_none() {
                    errors.push(BuildError::OrphanEdgeUse(EdgeUseId(index as u32)));
                }
            }
            for (index, owner) in self.wire_face.iter().enumerate() {
                if owner.is_none() {
                    errors.push(BuildError::OrphanWire(WireId(index as u32)));
                }
            }
            for (index, owner) in self.face_shell.iter().enumerate() {
                if owner.is_none() {
                    errors.push(BuildError::OrphanFace(FaceId(index as u32)));
                }
            }
            if !errors.is_empty() {
                return Err(ValidationReport { errors });
            }
            for edge_use in &mut self.edge_uses {
                edge_use.image_certificate = true;
            }
            let face_count = self.faces.len();
            let certified_prisms = self
                .solids
                .iter()
                .enumerate()
                .map(|(index, solid)| {
                    if let Some(certificate) = &self.retained_solid_certificates[index] {
                        Ok(certificate.prism.clone())
                    } else if solid.voids.is_empty() {
                        self.certified_prism_shell(solid.outer)
                    } else {
                        Ok(None)
                    }
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| ValidationReport {
                    errors: vec![error],
                })?;
            let certified_cylinders = self
                .solids
                .iter()
                .enumerate()
                .map(|(index, solid)| {
                    if let Some(certificate) = &self.retained_solid_certificates[index] {
                        Ok(certificate.cylinder.clone())
                    } else {
                        self.certified_cylinder_solid(solid)
                    }
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| ValidationReport {
                    errors: vec![error],
                })?;
            let certified_lofts = self
                .solids
                .iter()
                .enumerate()
                .map(|(index, solid)| {
                    if let Some(certificate) = &self.retained_solid_certificates[index] {
                        Ok(certificate.loft.clone())
                    } else if solid.voids.is_empty() && certified_prisms[index].is_none() {
                        self.certified_loft_shell(solid.outer)
                    } else {
                        Ok(None)
                    }
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| ValidationReport {
                    errors: vec![error],
                })?;
            let certified_curve_sweeps = self
                .solids
                .iter()
                .enumerate()
                .map(|(index, solid)| {
                    if let Some(certificate) = &self.retained_solid_certificates[index] {
                        Ok(certificate.curve_sweep.clone())
                    } else if solid.voids.is_empty() {
                        self.certified_curve_sweep_shell(solid.outer)
                    } else {
                        Ok(None)
                    }
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| ValidationReport {
                    errors: vec![error],
                })?;
            let certified_spheres = self
                .solids
                .iter()
                .enumerate()
                .map(|(index, solid)| {
                    if let Some(certificate) = &self.retained_solid_certificates[index] {
                        Ok(certificate.sphere.clone())
                    } else {
                        self.certified_sphere_solid(solid)
                    }
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| ValidationReport {
                    errors: vec![error],
                })?;
            let certified_sphere_pairs = self
                .solids
                .iter()
                .enumerate()
                .map(|(index, solid)| {
                    if let Some(certificate) = &self.retained_solid_certificates[index] {
                        Ok(certificate.sphere_pair.clone())
                    } else if solid.voids.is_empty() {
                        self.certified_sphere_pair_shell(solid.outer)
                    } else {
                        Ok(None)
                    }
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| ValidationReport {
                    errors: vec![error],
                })?;
            let certified_cone_frustums = self
                .solids
                .iter()
                .enumerate()
                .map(|(index, solid)| {
                    if let Some(certificate) = &self.retained_solid_certificates[index] {
                        Ok(certificate.cone_frustum.clone())
                    } else if solid.voids.is_empty() {
                        self.certified_cone_frustum_shell(solid.outer)
                    } else {
                        Ok(None)
                    }
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| ValidationReport {
                    errors: vec![error],
                })?;
            let certified_tori = self
                .solids
                .iter()
                .enumerate()
                .map(|(index, solid)| {
                    if let Some(certificate) = &self.retained_solid_certificates[index] {
                        Ok(certificate.torus.clone())
                    } else if solid.voids.is_empty() {
                        self.certified_torus_shell(solid.outer)
                    } else {
                        Ok(None)
                    }
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| ValidationReport {
                    errors: vec![error],
                })?;
            let certified_revolutions = self
                .solids
                .iter()
                .enumerate()
                .map(|(index, solid)| {
                    if let Some(certificate) = &self.retained_solid_certificates[index] {
                        Ok(certificate.revolution.clone())
                    } else {
                        self.certified_revolution_solid(solid)
                    }
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| ValidationReport {
                    errors: vec![error],
                })?;
            Ok(ArenaModel {
                data: Arc::new(ModelData {
                    vertices: self.vertices,
                    curves: self.curves,
                    pcurves: self.pcurves,
                    surfaces: self.surfaces,
                    edges: self.edges,
                    edge_uses: self.edge_uses,
                    wires: self.wires,
                    faces: self.faces,
                    shells: self.shells,
                    solids: self.solids,
                    vertex_edges: self.vertex_edges,
                    edge_uses_by_edge: self.edge_uses_by_edge,
                    edge_use_wire: self.edge_use_wire.into_iter().flatten().collect(),
                    wire_face: self.wire_face.into_iter().flatten().collect(),
                    face_shell: self.face_shell.into_iter().flatten().collect(),
                    shell_solid: self.shell_solid,
                    solid_certificates_valid: vec![true; certified_cylinders.len()],
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
                    face_contours: (0..face_count).map(|_| OnceLock::new()).collect(),
                }),
            })
        })
    }

    pub(in crate::topology) fn vertex_ref(&self, id: VertexId) -> Result<&Vertex, BuildError> {
        self.vertices
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Vertex,
                index: id.index(),
            })
    }

    pub(in crate::topology) fn curve_ref(&self, id: Curve3Id) -> Result<&Curve3, BuildError> {
        self.curves
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Curve3,
                index: id.index(),
            })
    }

    pub(in crate::topology) fn pcurve_ref(&self, id: PcurveId) -> Result<&Pcurve, BuildError> {
        self.pcurves
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Pcurve,
                index: id.index(),
            })
    }

    pub(in crate::topology) fn surface_ref(&self, id: SurfaceId) -> Result<&Surface, BuildError> {
        self.surfaces
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Surface,
                index: id.index(),
            })
    }

    pub(in crate::topology) fn edge_ref(&self, id: EdgeId) -> Result<&Edge, BuildError> {
        self.edges
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Edge,
                index: id.index(),
            })
    }

    pub(in crate::topology) fn edge_use_ref(&self, id: EdgeUseId) -> Result<&EdgeUse, BuildError> {
        self.edge_uses
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::EdgeUse,
                index: id.index(),
            })
    }

    pub(in crate::topology) fn wire_ref(&self, id: WireId) -> Result<&Wire, BuildError> {
        self.wires
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Wire,
                index: id.index(),
            })
    }

    pub(in crate::topology) fn face_ref(&self, id: FaceId) -> Result<&Face, BuildError> {
        self.faces
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Face,
                index: id.index(),
            })
    }

    pub(in crate::topology) fn shell_ref(&self, id: ShellId) -> Result<&Shell, BuildError> {
        self.shells
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Shell,
                index: id.index(),
            })
    }

    pub(in crate::topology) fn directed_vertices(
        &self,
        edge_use: EdgeUseId,
    ) -> Result<(VertexId, VertexId), BuildError> {
        let edge_use = self.edge_use_ref(edge_use)?;
        let edge = self.edge_ref(edge_use.edge)?;
        Ok(match edge_use.direction {
            Direction::Forward => (edge.start, edge.end),
            Direction::Reversed => (edge.end, edge.start),
        })
    }
}
