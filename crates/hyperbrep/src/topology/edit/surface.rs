//! Exact model edit surface operations.

use crate::topology::*;

impl Model {
    /// Splits a face along one retained exact surface curve.
    ///
    /// The supplied pcurve is materialized without inverse fitting and retains
    /// its exact affine correspondence to the spatial curve parameter. Curve
    /// endpoints on a trimmed face attach to existing outer-boundary vertices
    /// or split the unique containing canonical edges. A closed latitude on a
    /// boundaryless whole sphere authors two complementary periodic caps. The
    /// new canonical curve is shared by two opposite edge uses, and the
    /// complete model is revalidated before publication.
    pub fn split_face_by_surface_curve(
        &self,
        face_id: FaceId,
        intersection: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
    ) -> Result<(Self, SurfaceCurveFaceSplit), TopologyEditError> {
        let curve = intersection.curve();
        let pcurve = intersection.pcurve(operand);
        let image_certificate = intersection.has_image_certificate(operand);
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?;
        let start = curve.start()?;
        let end = curve.end()?;
        if self.retained_surface_curve_endpoints_equal(
            face_id,
            intersection,
            operand,
            &start,
            &end,
        )? {
            let (canonical_curve, canonical_pcurve);
            let reversed = curve.reversed()?;
            let (curve, pcurve) =
                if compare_curve3_exact_data(&curve.exact_data(), &reversed.exact_data())?
                    == std::cmp::Ordering::Greater
                {
                    canonical_curve = curve.reversed()?;
                    canonical_pcurve = pcurve.reversed()?;
                    (&canonical_curve, &canonical_pcurve)
                } else {
                    (curve, pcurve)
                };
            let (model, split) = if face.outer().is_none() {
                self.split_whole_sphere_by_surface_curve(face_id, curve, pcurve, image_certificate)?
            } else if self
                .surface(face.surface)
                .expect("validated face surface")
                .kind()
                == SurfaceKind::Sphere
                && (face.inner().is_empty()
                    && self.is_spherical_latitude_wire(
                        face.outer().expect("trimmed face carries outer wire"),
                    )?
                    || !face.inner().is_empty())
            {
                self.split_spherical_cap_by_surface_curve(
                    face_id,
                    curve,
                    pcurve,
                    image_certificate,
                )?
            } else {
                self.split_face_by_closed_surface_curve(face_id, curve, pcurve, image_certificate)?
            };
            return Ok((model, SurfaceCurveFaceSplit::Closed(split)));
        }
        let Some(_) = face.outer() else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        let materialized = pcurve.materialize()?;
        let correspondence = match materialized.correspondence() {
            SurfacePcurveCorrespondence::Affine { scale, offset } => {
                ParameterCorrespondence::affine(scale.clone(), offset.clone())?
            }
            SurfacePcurveCorrespondence::AngularSweep { .. } => {
                ParameterCorrespondence::angular_sweep()
            }
        };
        let (staged, start_vertex, start_edge) = self.attach_surface_curve_split_endpoint(
            face_id,
            Endpoint::Start,
            &start,
            intersection.start_boundary_contacts(operand),
        )?;
        let (staged, end_vertex, end_edge) = staged.attach_surface_curve_split_endpoint(
            face_id,
            Endpoint::End,
            &end,
            intersection.end_boundary_contacts(operand),
        )?;
        let (staged, face) = staged.split_face_with_geometry(
            face_id,
            start_vertex,
            end_vertex,
            Some((
                curve.clone(),
                Pcurve::new(materialized.curve().clone()),
                correspondence,
                image_certificate,
            )),
        )?;
        Ok((
            staged,
            SurfaceCurveFaceSplit::Open(CurveFaceSplit {
                start_edge,
                end_edge,
                face,
            }),
        ))
    }

    pub(in crate::topology) fn split_whole_sphere_by_surface_curve(
        &self,
        face_id: FaceId,
        curve: &Curve3,
        pcurve: &SurfaceIntersectionPcurve,
        image_certificate: bool,
    ) -> Result<(Self, ClosedSurfaceCurveFaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?
            .clone();
        if !face.is_whole_surface()
            || self
                .surface(face.surface)
                .expect("validated whole-face surface")
                .kind()
                != SurfaceKind::Sphere
        {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        }
        let start = curve.start()?;
        let end = curve.end()?;
        if exact_point_order(&start, &end)? != std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }
        let midpoint = ((curve.domain().start() + curve.domain().end()) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let midpoint_point = curve.point_at(&midpoint)?;
        if exact_point_order(&start, &midpoint_point)? == std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }

        let ranges = [
            (curve.domain().start(), &midpoint),
            (&midpoint, curve.domain().end()),
        ];
        let mut halves = Vec::with_capacity(2);
        for (range_start, range_end) in ranges {
            let spatial = curve.subcurve(range_start, range_end)?;
            let retained = pcurve.subcurve(range_start, range_end)?;
            let materialized = retained.materialize()?;
            let forward_pcurve = Pcurve::new(materialized.curve().clone());
            let reverse_pcurve = forward_pcurve.reversed()?;
            let (forward_correspondence, reverse_correspondence) = match materialized
                .correspondence()
            {
                SurfacePcurveCorrespondence::Affine { scale, offset } => {
                    let source_span = range_end - range_start;
                    let spatial_span = spatial.domain().end() - spatial.domain().start();
                    let domain_scale = (spatial_span / source_span)
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let edge_scale = &domain_scale * scale;
                    let edge_offset =
                        spatial.domain().start() + &domain_scale * (offset - range_start);
                    (
                        ParameterCorrespondence::affine(edge_scale.clone(), edge_offset.clone())?,
                        ParameterCorrespondence::affine(
                            -edge_scale.clone(),
                            edge_scale
                                * (forward_pcurve.domain_start()? + forward_pcurve.domain_end()?)
                                + edge_offset,
                        )?,
                    )
                }
                SurfacePcurveCorrespondence::AngularSweep { .. } => (
                    ParameterCorrespondence::angular_sweep(),
                    ParameterCorrespondence::angular_sweep(),
                ),
            };
            halves.push((
                spatial,
                forward_pcurve,
                forward_correspondence,
                reverse_pcurve,
                reverse_correspondence,
            ));
        }

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        let seam_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: start.clone(),
        });
        let midpoint_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: midpoint_point,
        });

        let mut edges = Vec::with_capacity(2);
        let mut forward_uses = Vec::with_capacity(2);
        let mut reverse_uses = Vec::with_capacity(2);
        for (index, (spatial, forward, forward_map, reverse, reverse_map)) in
            halves.into_iter().enumerate()
        {
            let curve_id = Curve3Id::from_index(data.curves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
            let domain = spatial.domain().clone();
            data.curves.push(spatial);
            let edge = EdgeId::from_index(data.edges.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
            data.edges.push(Edge {
                start: if index == 0 {
                    seam_vertex
                } else {
                    midpoint_vertex
                },
                end: if index == 0 {
                    midpoint_vertex
                } else {
                    seam_vertex
                },
                curve: curve_id,
                domain,
                endpoint_certificate: false,
            });
            edges.push(edge);

            let forward_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(forward);
            let forward_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Forward,
                pcurve: forward_pcurve,
                parameter_correspondence: forward_map,
                image_certificate,
            });
            forward_uses.push(forward_use);

            let reverse_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(reverse);
            let reverse_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Reversed,
                pcurve: reverse_pcurve,
                parameter_correspondence: reverse_map,
                image_certificate,
            });
            reverse_uses.push(reverse_use);
        }
        reverse_uses.reverse();
        let first_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: forward_uses.clone(),
            simplicity_certificate: true,
        });
        let second_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: reverse_uses.clone(),
            simplicity_certificate: true,
        });
        data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
            outer: first_wire,
            inner: Vec::new(),
        };
        let second_face = FaceId::from_index(data.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        data.faces.push(Face {
            surface: face.surface,
            orientation: face.orientation,
            boundary: FaceBoundary::Trimmed {
                outer: second_wire,
                inner: Vec::new(),
            },
        });
        let shell = self.data.face_shell[face_id.index()];
        let source_position = data.shells[shell.index()]
            .faces
            .iter()
            .position(|candidate| *candidate == face_id)
            .expect("validated face-shell adjacency");
        data.shells[shell.index()]
            .faces
            .insert(source_position + 1, second_face);
        reset_model_caches(data);
        let validated = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;
        Ok((
            validated,
            ClosedSurfaceCurveFaceSplit {
                seam_vertex,
                midpoint_vertex,
                edges: [edges[0], edges[1]],
                first_edge_uses: [forward_uses[0], forward_uses[1]],
                second_edge_uses: [reverse_uses[0], reverse_uses[1]],
                first_wire,
                second_wire,
                first_face: face_id,
                second_face,
            },
        ))
    }

    pub(in crate::topology) fn is_spherical_latitude_wire(
        &self,
        wire: WireId,
    ) -> Result<bool, TopologyEditError> {
        let wire = self.wire(wire).expect("validated spherical wire");
        let mut latitude = None;
        for edge_use_id in &wire.edge_uses {
            let edge_use = self
                .edge_use(*edge_use_id)
                .expect("validated spherical edge use");
            let Some(line) = self
                .pcurve(edge_use.pcurve)
                .expect("validated spherical pcurve")
                .line_segment()
            else {
                return Ok(false);
            };
            if !real_values_equal(line.start().y(), line.end().y())? {
                return Ok(false);
            }
            if let Some(expected) = &latitude {
                if !real_values_equal(line.start().y(), expected)? {
                    return Ok(false);
                }
            } else {
                latitude = Some(line.start().y().clone());
            }
        }
        Ok(latitude.is_some())
    }

    pub(in crate::topology) fn split_spherical_cap_by_surface_curve(
        &self,
        face_id: FaceId,
        curve: &Curve3,
        pcurve: &SurfaceIntersectionPcurve,
        image_certificate: bool,
    ) -> Result<(Self, ClosedSurfaceCurveFaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?
            .clone();
        let FaceBoundary::Trimmed { outer, inner } = &face.boundary else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        if !inner.is_empty() {
            return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
        }
        let old_wire = self.wire(*outer).expect("validated spherical cap wire");
        let old_line = self
            .pcurve(
                self.edge_use(old_wire.edge_uses[0])
                    .expect("validated spherical use")
                    .pcurve,
            )
            .expect("validated spherical pcurve")
            .line_segment()
            .expect("validated spherical latitude");
        let old_latitude = old_line.start().y().clone();
        let increasing = decided_model_order(compare_reals(
            old_line.end().x(),
            old_line.start().x(),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Greater;
        let upper = match face.orientation {
            Orientation::Forward => increasing,
            Orientation::Reversed => !increasing,
        };
        let materialized = pcurve.materialize()?;
        if materialized.curve().family() != CurveFamily2::Line {
            return Err(TopologyEditError::UnsupportedFaceSplitCurve(curve.kind()));
        }
        let new_start = materialized.curve().start();
        let new_end = materialized.curve().end();
        if !real_values_equal(
            (new_start)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .y(),
            (new_end)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .y(),
        )? {
            return Err(TopologyEditError::UnsupportedFaceSplitCurve(curve.kind()));
        }
        let new_latitude = (new_start)
            .coordinates()
            .ok_or(GeometryError::UnsupportedPcurveContour)?
            .y()
            .clone();
        let relation = decided_model_order(compare_reals(
            &new_latitude,
            &old_latitude,
            crate::STRICT_PREDICATES,
        ))?;
        if (upper && relation != std::cmp::Ordering::Greater)
            || (!upper && relation != std::cmp::Ordering::Less)
        {
            return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
        }

        let start = curve.start()?;
        let midpoint = ((curve.domain().start() + curve.domain().end()) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let midpoint_point = curve.point_at(&midpoint)?;
        let ranges = [
            (curve.domain().start(), &midpoint),
            (&midpoint, curve.domain().end()),
        ];
        let mut halves = Vec::with_capacity(2);
        for (range_start, range_end) in ranges {
            let spatial = curve.subcurve(range_start, range_end)?;
            let retained = pcurve.subcurve(range_start, range_end)?;
            let materialized = retained.materialize()?;
            let forward_pcurve = Pcurve::new(materialized.curve().clone());
            let reverse_pcurve = forward_pcurve.reversed()?;
            let (forward_correspondence, reverse_correspondence) = match materialized
                .correspondence()
            {
                SurfacePcurveCorrespondence::Affine { scale, offset } => {
                    let source_span = range_end - range_start;
                    let spatial_span = spatial.domain().end() - spatial.domain().start();
                    let domain_scale = (spatial_span / source_span)
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let edge_scale = &domain_scale * scale;
                    let edge_offset =
                        spatial.domain().start() + &domain_scale * (offset - range_start);
                    (
                        ParameterCorrespondence::affine(edge_scale.clone(), edge_offset.clone())?,
                        ParameterCorrespondence::affine(
                            -edge_scale.clone(),
                            edge_scale
                                * (forward_pcurve.domain_start()? + forward_pcurve.domain_end()?)
                                + edge_offset,
                        )?,
                    )
                }
                SurfacePcurveCorrespondence::AngularSweep { .. } => (
                    ParameterCorrespondence::angular_sweep(),
                    ParameterCorrespondence::angular_sweep(),
                ),
            };
            halves.push((
                spatial,
                forward_pcurve,
                forward_correspondence,
                reverse_pcurve,
                reverse_correspondence,
            ));
        }

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        let seam_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: start.clone(),
        });
        let midpoint_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: midpoint_point,
        });
        let mut edges = Vec::with_capacity(2);
        let mut forward_uses = Vec::with_capacity(2);
        let mut reverse_uses = Vec::with_capacity(2);
        for (index, (spatial, forward, forward_map, reverse, reverse_map)) in
            halves.into_iter().enumerate()
        {
            let curve_id = Curve3Id::from_index(data.curves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
            let domain = spatial.domain().clone();
            data.curves.push(spatial);
            let edge = EdgeId::from_index(data.edges.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
            data.edges.push(Edge {
                start: if index == 0 {
                    seam_vertex
                } else {
                    midpoint_vertex
                },
                end: if index == 0 {
                    midpoint_vertex
                } else {
                    seam_vertex
                },
                curve: curve_id,
                domain,
                endpoint_certificate: false,
            });
            edges.push(edge);
            let forward_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(forward);
            let forward_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Forward,
                pcurve: forward_pcurve,
                parameter_correspondence: forward_map,
                image_certificate,
            });
            forward_uses.push(forward_use);
            let reverse_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(reverse);
            let reverse_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Reversed,
                pcurve: reverse_pcurve,
                parameter_correspondence: reverse_map,
                image_certificate,
            });
            reverse_uses.push(reverse_use);
        }
        reverse_uses.reverse();
        let forward_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: forward_uses.clone(),
            simplicity_certificate: true,
        });
        let reverse_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: reverse_uses.clone(),
            simplicity_certificate: true,
        });

        let second_face = FaceId::from_index(data.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        if upper {
            data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
                outer: *outer,
                inner: vec![reverse_wire],
            };
            data.faces.push(Face {
                surface: face.surface,
                orientation: face.orientation,
                boundary: FaceBoundary::Trimmed {
                    outer: forward_wire,
                    inner: Vec::new(),
                },
            });
        } else {
            data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
                outer: reverse_wire,
                inner: Vec::new(),
            };
            data.faces.push(Face {
                surface: face.surface,
                orientation: face.orientation,
                boundary: FaceBoundary::Trimmed {
                    outer: forward_wire,
                    inner: vec![*outer],
                },
            });
        }
        let shell = self.data.face_shell[face_id.index()];
        let source_position = data.shells[shell.index()]
            .faces
            .iter()
            .position(|candidate| *candidate == face_id)
            .expect("validated face-shell adjacency");
        data.shells[shell.index()]
            .faces
            .insert(source_position + 1, second_face);
        reset_model_caches(data);
        let validated = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;
        Ok((
            validated,
            ClosedSurfaceCurveFaceSplit {
                seam_vertex,
                midpoint_vertex,
                edges: [edges[0], edges[1]],
                first_edge_uses: [reverse_uses[0], reverse_uses[1]],
                second_edge_uses: [forward_uses[0], forward_uses[1]],
                first_wire: reverse_wire,
                second_wire: forward_wire,
                first_face: face_id,
                second_face,
            },
        ))
    }

    pub(in crate::topology) fn split_face_by_closed_surface_curve(
        &self,
        face_id: FaceId,
        curve: &Curve3,
        pcurve: &SurfaceIntersectionPcurve,
        image_certificate: bool,
    ) -> Result<(Self, ClosedSurfaceCurveFaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?
            .clone();
        let FaceBoundary::Trimmed { outer, inner } = &face.boundary else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        let start = curve.start()?;
        let end = curve.end()?;
        if exact_point_order(&start, &end)? != std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }
        let midpoint = ((curve.domain().start() + curve.domain().end()) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let midpoint_point = curve.point_at(&midpoint)?;
        if exact_point_order(&start, &midpoint_point)? == std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }

        let materialized_loop = pcurve.materialize()?;
        let loop_path = CurvePath2::try_new(vec![materialized_loop.curve().clone()])
            .map_err(GeometryError::from)?;
        let loop_area = crate::error::classified(
            loop_path
                .boundary_loop()
                .map_err(GeometryError::from)?
                .signed_area(),
        )
        .map_err(GeometryError::from)?;
        let loop_area = resolve_planar_classification(loop_area)?
            .ok_or(GeometryError::UnsupportedPcurveContour)?;
        let area_order = decided_model_order(compare_reals(
            &loop_area,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))?;
        if area_order == std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }
        let expected_outer_order = match face.orientation {
            Orientation::Forward => std::cmp::Ordering::Greater,
            Orientation::Reversed => std::cmp::Ordering::Less,
        };
        let interior_forward = area_order == expected_outer_order;

        let loop_start = materialized_loop.curve().start();
        let classify = |path: &CurvePath2,
                        point: &hypercurve::CurvePoint2|
         -> Result<ContourPointLocation, TopologyEditError> {
            match crate::error::classified(path.classify_point(point))
                .map_err(GeometryError::from)?
            {
                Classification::Decided(location) => Ok(location),
                Classification::Uncertain(reason) => {
                    Err(GeometryError::PlanarClassificationUnresolved(reason).into())
                }
            }
        };
        let outer_path = self.build_model_wire_curve_path(*outer)?;
        if classify(&outer_path, &loop_start)? != ContourPointLocation::Inside {
            return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
        }
        for hole in inner {
            let hole_path = self.build_model_wire_curve_path(*hole)?;
            if classify(&hole_path, &loop_start)? != ContourPointLocation::Outside {
                return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
            }
        }

        let ranges = [
            (curve.domain().start(), &midpoint),
            (&midpoint, curve.domain().end()),
        ];
        let mut halves = Vec::with_capacity(2);
        for (range_start, range_end) in ranges {
            let spatial = curve.subcurve(range_start, range_end)?;
            let retained = pcurve.subcurve(range_start, range_end)?;
            let materialized = retained.materialize()?;
            let forward_pcurve = Pcurve::new(materialized.curve().clone());
            let reverse_pcurve = forward_pcurve.reversed()?;
            let (forward_correspondence, reverse_correspondence) = match materialized
                .correspondence()
            {
                SurfacePcurveCorrespondence::Affine { scale, offset } => {
                    let source_span = range_end - range_start;
                    let spatial_span = spatial.domain().end() - spatial.domain().start();
                    let domain_scale = (spatial_span / source_span)
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let edge_scale = &domain_scale * scale;
                    let edge_offset =
                        spatial.domain().start() + &domain_scale * (offset - range_start);
                    (
                        ParameterCorrespondence::affine(edge_scale.clone(), edge_offset.clone())?,
                        ParameterCorrespondence::affine(
                            -edge_scale.clone(),
                            edge_scale
                                * (forward_pcurve.domain_start()? + forward_pcurve.domain_end()?)
                                + edge_offset,
                        )?,
                    )
                }
                SurfacePcurveCorrespondence::AngularSweep { .. } => (
                    ParameterCorrespondence::angular_sweep(),
                    ParameterCorrespondence::angular_sweep(),
                ),
            };
            halves.push((
                spatial,
                forward_pcurve,
                forward_correspondence,
                reverse_pcurve,
                reverse_correspondence,
            ));
        }

        let mut enclosed_inner = Vec::new();
        let mut exterior_inner = Vec::new();
        for hole in inner {
            let hole_path = self.build_model_wire_curve_path(*hole)?;
            let representative = hole_path.start();
            match classify(&loop_path, &representative)? {
                ContourPointLocation::Inside => enclosed_inner.push(*hole),
                ContourPointLocation::Outside => exterior_inner.push(*hole),
                ContourPointLocation::Boundary => {
                    return Err(BuildError::IntersectingFaceWires {
                        first: *outer,
                        second: *hole,
                    }
                    .into());
                }
            }
        }

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        let seam_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: start.clone(),
        });
        let midpoint_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: midpoint_point,
        });

        let mut edges = Vec::with_capacity(2);
        let mut forward_uses = Vec::with_capacity(2);
        let mut reverse_uses = Vec::with_capacity(2);
        for (index, (spatial, forward, forward_map, reverse, reverse_map)) in
            halves.into_iter().enumerate()
        {
            let curve_id = Curve3Id::from_index(data.curves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
            let domain = spatial.domain().clone();
            data.curves.push(spatial);
            let edge = EdgeId::from_index(data.edges.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
            data.edges.push(Edge {
                start: if index == 0 {
                    seam_vertex
                } else {
                    midpoint_vertex
                },
                end: if index == 0 {
                    midpoint_vertex
                } else {
                    seam_vertex
                },
                curve: curve_id,
                domain,
                endpoint_certificate: false,
            });
            edges.push(edge);

            let forward_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(forward);
            let forward_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Forward,
                pcurve: forward_pcurve,
                parameter_correspondence: forward_map,
                image_certificate,
            });
            forward_uses.push(forward_use);

            let reverse_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(reverse);
            let reverse_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Reversed,
                pcurve: reverse_pcurve,
                parameter_correspondence: reverse_map,
                image_certificate,
            });
            reverse_uses.push(reverse_use);
        }
        reverse_uses.reverse();
        let (interior_uses, exterior_uses) = if interior_forward {
            (forward_uses, reverse_uses)
        } else {
            (reverse_uses, forward_uses)
        };

        let exterior_inner_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: exterior_uses.clone(),
            simplicity_certificate: true,
        });
        let interior_outer_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: interior_uses.clone(),
            simplicity_certificate: true,
        });
        exterior_inner.push(exterior_inner_wire);
        data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
            outer: *outer,
            inner: exterior_inner,
        };
        let second_face = FaceId::from_index(data.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        data.faces.push(Face {
            surface: face.surface,
            orientation: face.orientation,
            boundary: FaceBoundary::Trimmed {
                outer: interior_outer_wire,
                inner: enclosed_inner,
            },
        });
        let shell = self.data.face_shell[face_id.index()];
        let source_position = data.shells[shell.index()]
            .faces
            .iter()
            .position(|candidate| *candidate == face_id)
            .expect("validated face-shell adjacency");
        data.shells[shell.index()]
            .faces
            .insert(source_position + 1, second_face);
        reset_model_caches(data);
        let validated = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;
        Ok((
            validated,
            ClosedSurfaceCurveFaceSplit {
                seam_vertex,
                midpoint_vertex,
                edges: [edges[0], edges[1]],
                first_edge_uses: [exterior_uses[0], exterior_uses[1]],
                second_edge_uses: [interior_uses[0], interior_uses[1]],
                first_wire: exterior_inner_wire,
                second_wire: interior_outer_wire,
                first_face: face_id,
                second_face,
            },
        ))
    }
}
