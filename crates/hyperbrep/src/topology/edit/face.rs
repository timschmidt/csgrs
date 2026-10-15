//! Exact model edit face operations.

use crate::topology::*;

impl ArenaModel {
    /// Splits a trimmed planar face along one exact straight curve fragment.
    ///
    /// Each fragment endpoint is attached by mathematical identity: an
    /// existing outer-boundary vertex is reused, otherwise the unique
    /// containing canonical edge is split at its exact represented parameter.
    /// The resulting boundary vertices are then passed through
    /// [`ArenaModel::split_face`], so the chord is shared by identity and the
    /// complete model is revalidated before publication.
    pub fn split_face_by_curve(
        &self,
        face_id: FaceId,
        curve: &Curve3,
    ) -> Result<(Self, ArenaCurveFaceSplit), TopologyEditError> {
        crate::context::strict(|| {
            let face = self
                .face(face_id)
                .ok_or(TopologyEditError::InvalidReference {
                    kind: EntityKind::Face,
                    index: face_id.index(),
                })?;
            let Some(_) = face.outer() else {
                return Err(TopologyEditError::WholeSurfaceFace(face_id));
            };
            let surface = self
                .surface(face.surface)
                .expect("validated face surface ID");
            if surface.kind() != SurfaceKind::Plane {
                return Err(TopologyEditError::UnsupportedFaceSplitSurface(
                    surface.kind(),
                ));
            }
            if curve.kind() != Curve3Kind::Line {
                return Err(TopologyEditError::UnsupportedFaceSplitCurve(curve.kind()));
            }

            let start = curve.start()?;
            let end = curve.end()?;
            let (staged, start_vertex, start_edge) =
                self.attach_face_split_endpoint(face_id, Endpoint::Start, &start)?;
            let (staged, end_vertex, end_edge) =
                staged.attach_face_split_endpoint(face_id, Endpoint::End, &end)?;
            let (staged, face) = staged.split_face(face_id, start_vertex, end_vertex)?;
            Ok((
                staged,
                ArenaCurveFaceSplit {
                    start_edge,
                    end_edge,
                    face,
                },
            ))
        })
    }

    pub(in crate::topology) fn attach_face_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &Point3,
    ) -> Result<(Self, VertexId, Option<ArenaEdgeSplit>), TopologyEditError> {
        match self.locate_face_split_endpoint(face_id, endpoint, point)? {
            FaceSplitEndpointLocation::Vertex(vertex) => Ok((self.clone(), vertex, None)),
            FaceSplitEndpointLocation::Edge {
                edge, parameter, ..
            } => {
                let (staged, split) = self.split_edge(edge, (parameter).into())?;
                Ok((staged, split.vertex, Some(split)))
            }
        }
    }

    pub(in crate::topology) fn attach_face_boundary_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &Point3,
    ) -> Result<(Self, WireId, VertexId, Option<ArenaEdgeSplit>), TopologyEditError> {
        let (wire, location) =
            self.locate_face_boundary_split_endpoint(face_id, endpoint, point)?;
        match location {
            FaceSplitEndpointLocation::Vertex(vertex) => Ok((self.clone(), wire, vertex, None)),
            FaceSplitEndpointLocation::Edge {
                edge, parameter, ..
            } => {
                let (staged, split) = self.split_edge(edge, (parameter).into())?;
                Ok((staged, wire, split.vertex, Some(split)))
            }
        }
    }

    pub(in crate::topology) fn locate_face_boundary_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &Point3,
    ) -> Result<(WireId, FaceSplitEndpointLocation), TopologyEditError> {
        let face = self
            .face(face_id)
            .expect("surface split prevalidates the face");
        let mut boundary_vertices = Vec::<(WireId, VertexId)>::new();
        let mut boundary_edges = Vec::<(WireId, EdgeId)>::new();
        for wire_id in face.boundary_wires() {
            let wire = self.wire(*wire_id).expect("validated face boundary wire");
            for use_id in wire.edge_uses() {
                let edge_use = self.edge_use(*use_id).expect("validated edge use");
                let edge = self.edge(edge_use.edge).expect("validated canonical edge");
                for vertex in [edge.start, edge.end] {
                    if !boundary_vertices.contains(&(*wire_id, vertex)) {
                        boundary_vertices.push((*wire_id, vertex));
                    }
                }
                if !boundary_edges.contains(&(*wire_id, edge_use.edge)) {
                    boundary_edges.push((*wire_id, edge_use.edge));
                }
            }
        }
        let matching_vertices = boundary_vertices
            .into_iter()
            .filter_map(|(wire, vertex)| {
                self.vertex(vertex)
                    .expect("validated boundary vertex")
                    .point
                    .coincides_with(&point.clone().into())
                    .map_err(BuildError::from)
                    .map(|equal| equal.then_some((wire, vertex)))
                    .transpose()
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(TopologyEditError::Build)?;
        match matching_vertices.as_slice() {
            [(wire, vertex)] => {
                return Ok((*wire, FaceSplitEndpointLocation::Vertex(*vertex)));
            }
            [] => {}
            _ => {
                return Err(TopologyEditError::FaceSplitEndpointAmbiguous {
                    face: face_id,
                    endpoint,
                });
            }
        }

        let mut locations = Vec::new();
        for (wire, edge_id) in boundary_edges {
            let edge = self.edge(edge_id).expect("validated canonical edge");
            let curve = self.curve(edge.curve).expect("validated edge curve");
            let CurveParameterLocation::Parameters(parameters) = curve.parameters_of(point)? else {
                continue;
            };
            for parameter in parameters {
                let parameter = parameter
                    .try_real()
                    .cloned()
                    .ok_or(GeometryError::UnrepresentableParameter)?;
                if edge
                    .domain
                    .contains(&crate::Parameter::from_real(parameter.clone()))?
                    && decided_model_order(compare_reals(
                        &parameter,
                        (edge.domain.start())
                            .try_real()
                            .ok_or(crate::GeometryError::UnrepresentableParameter)?,
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Greater
                    && decided_model_order(compare_reals(
                        &parameter,
                        (edge.domain.end())
                            .try_real()
                            .ok_or(crate::GeometryError::UnrepresentableParameter)?,
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Less
                {
                    locations.push((
                        wire,
                        FaceSplitEndpointLocation::Edge {
                            edge: edge_id,
                            parameter,
                            pcurve_split: None,
                        },
                    ));
                }
            }
        }
        match locations.len() {
            1 => Ok(locations.pop().expect("one boundary location")),
            0 => Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary {
                face: face_id,
                endpoint,
            }),
            _ => Err(TopologyEditError::FaceSplitEndpointAmbiguous {
                face: face_id,
                endpoint,
            }),
        }
    }

    pub(in crate::topology) fn locate_face_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &Point3,
    ) -> Result<FaceSplitEndpointLocation, TopologyEditError> {
        let face = self
            .face(face_id)
            .expect("curve-driven split prevalidates the face");
        let outer = face
            .outer()
            .expect("curve-driven split prevalidates a trimmed face");
        let wire = self.wire(outer).expect("validated face outer wire");
        let mut boundary_vertices = Vec::new();
        let mut boundary_edges = Vec::new();
        for use_id in wire.edge_uses() {
            let edge_use = self.edge_use(*use_id).expect("validated edge use");
            let edge = self.edge(edge_use.edge).expect("validated canonical edge");
            for vertex in [edge.start, edge.end] {
                if !boundary_vertices.contains(&vertex) {
                    boundary_vertices.push(vertex);
                }
            }
            if !boundary_edges.contains(&edge_use.edge) {
                boundary_edges.push(edge_use.edge);
            }
        }
        let mut matching_vertices = Vec::new();
        for vertex in boundary_vertices {
            if matches!(
                self.vertex(vertex)
                    .expect("validated boundary vertex")
                    .point
                    .coincides_with(&point.clone().into()),
                Ok(true)
            ) {
                matching_vertices.push(vertex);
            }
        }
        match matching_vertices.as_slice() {
            [vertex] => return Ok(FaceSplitEndpointLocation::Vertex(*vertex)),
            [] => {}
            _ => {
                return Err(TopologyEditError::FaceSplitEndpointAmbiguous {
                    face: face_id,
                    endpoint,
                });
            }
        }

        let mut locations = Vec::new();
        for edge_id in boundary_edges {
            let edge = self.edge(edge_id).expect("validated canonical edge");
            let curve = self.curve(edge.curve).expect("validated edge curve");
            let parameters = match curve.parameters_of(point) {
                Ok(CurveParameterLocation::Parameters(parameters)) => parameters,
                Ok(_) | Err(GeometryError::PredicateUnresolved { .. }) => continue,
                Err(error) => return Err(error.into()),
            };
            for parameter in parameters {
                let parameter = parameter
                    .try_real()
                    .cloned()
                    .ok_or(GeometryError::UnrepresentableParameter)?;
                let after_start = compare_reals(
                    &parameter,
                    (edge.domain.start())
                        .try_real()
                        .ok_or(crate::GeometryError::UnrepresentableParameter)?,
                    crate::STRICT_PREDICATES,
                );
                let before_end = compare_reals(
                    &parameter,
                    (edge.domain.end())
                        .try_real()
                        .ok_or(crate::GeometryError::UnrepresentableParameter)?,
                    crate::STRICT_PREDICATES,
                );
                if matches!(
                    after_start,
                    PredicateOutcome::Decided {
                        value: std::cmp::Ordering::Greater,
                        ..
                    }
                ) && matches!(
                    before_end,
                    PredicateOutcome::Decided {
                        value: std::cmp::Ordering::Less,
                        ..
                    }
                ) {
                    locations.push((edge_id, parameter));
                }
            }
        }
        match locations.len() {
            0 => Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary {
                face: face_id,
                endpoint,
            }),
            1 => {
                let (edge, parameter) = locations.pop().expect("one exact edge location");
                Ok(FaceSplitEndpointLocation::Edge {
                    edge,
                    parameter,
                    pcurve_split: None,
                })
            }
            _ => Err(TopologyEditError::FaceSplitEndpointAmbiguous {
                face: face_id,
                endpoint,
            }),
        }
    }

    /// Splits one trimmed planar face between two nonadjacent outer-boundary
    /// vertices with an exact straight chord.
    ///
    /// The source face and outer-wire IDs are retained for the first result.
    /// The chord is one canonical edge with two opposite face-local uses.
    /// Existing holes are assigned to exactly one result by certified contour
    /// classification. The owning shell is updated, then the complete snapshot
    /// is replayed through [`ArenaModelBuilder`] before publication.
    pub fn split_face(
        &self,
        face_id: FaceId,
        start: VertexId,
        end: VertexId,
    ) -> Result<(Self, ArenaFaceSplit), TopologyEditError> {
        crate::context::strict(|| self.split_face_with_geometry(face_id, start, end, None))
    }

    pub(in crate::topology) fn split_face_with_geometry(
        &self,
        face_id: FaceId,
        start: VertexId,
        end: VertexId,
        authored: Option<(Curve3, Pcurve, ParameterCorrespondence, bool)>,
    ) -> Result<(Self, ArenaFaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?
            .clone();
        for vertex in [start, end] {
            if self.vertex(vertex).is_none() {
                return Err(TopologyEditError::InvalidReference {
                    kind: EntityKind::Vertex,
                    index: vertex.index(),
                });
            }
        }
        let (outer, inner) = match face.boundary {
            FaceBoundary::WholeSurface => {
                return Err(TopologyEditError::WholeSurfaceFace(face_id));
            }
            FaceBoundary::Trimmed { outer, inner } => (outer, inner),
        };
        let surface = self
            .surface(face.surface)
            .expect("validated face surface ID");
        let authored_curve = authored.is_some();
        if authored.is_none() && surface.kind() != SurfaceKind::Plane {
            return Err(TopologyEditError::UnsupportedFaceSplitSurface(
                surface.kind(),
            ));
        }
        let outer_uses = self
            .wire(outer)
            .expect("validated face outer-wire ID")
            .edge_uses
            .clone();
        let directed_start = |edge_use_id: EdgeUseId| {
            let edge_use = self
                .edge_use(edge_use_id)
                .expect("validated wire edge-use ID");
            let edge = self
                .edge(edge_use.edge)
                .expect("validated edge-use edge ID");
            match edge_use.direction {
                Direction::Forward => edge.start,
                Direction::Reversed => edge.end,
            }
        };
        let unique_boundary_index = |vertex: VertexId| -> Result<usize, TopologyEditError> {
            let matches = outer_uses
                .iter()
                .enumerate()
                .filter_map(|(index, edge_use)| {
                    (directed_start(*edge_use) == vertex).then_some(index)
                })
                .collect::<Vec<_>>();
            if matches.len() == 1 {
                Ok(matches[0])
            } else {
                Err(TopologyEditError::VertexNotOnOuterBoundary {
                    face: face_id,
                    vertex,
                })
            }
        };
        let start_index = unique_boundary_index(start)?;
        let end_index = unique_boundary_index(end)?;
        let use_count = outer_uses.len();
        let forward_distance = (end_index + use_count - start_index) % use_count;
        if start == end
            || (!authored_curve && (forward_distance <= 1 || forward_distance + 1 >= use_count))
            || (authored_curve && forward_distance == 0)
        {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }
        let cyclic_path = |from: usize, to: usize| {
            let mut path = Vec::new();
            let mut index = from;
            while index != to {
                path.push(outer_uses[index]);
                index = (index + 1) % use_count;
            }
            path
        };
        let mut first_path = cyclic_path(start_index, end_index);
        let mut second_path = cyclic_path(end_index, start_index);

        let (curve_geometry, forward_pcurve, forward_correspondence, image_certificate) =
            match authored {
                Some(authored) => authored,
                None => {
                    let pcurve_start =
                        |edge_use_id: EdgeUseId| -> Result<CurvePoint2, GeometryError> {
                            let edge_use = self
                                .edge_use(edge_use_id)
                                .expect("validated wire edge-use ID");
                            let pcurve = self
                                .pcurve(edge_use.pcurve)
                                .expect("validated edge-use pcurve ID");
                            Ok((pcurve.curve().start().clone())
                                .coordinates()
                                .ok_or(GeometryError::UnsupportedPcurveContour)?
                                .clone())
                        };
                    let start_uv = pcurve_start(outer_uses[start_index])?;
                    let end_uv = pcurve_start(outer_uses[end_index])?;
                    let start_point = self
                        .vertex(start)
                        .expect("outer-boundary vertex resolves")
                        .represented_point()?
                        .clone();
                    let end_point = self
                        .vertex(end)
                        .expect("outer-boundary vertex resolves")
                        .represented_point()?
                        .clone();
                    (
                        Curve3::line(start_point, end_point)?,
                        Pcurve::new(Curve2::from(
                            LineSeg2::try_new(start_uv, end_uv).map_err(GeometryError::from)?,
                        )),
                        ParameterCorrespondence::identity(),
                        // The source boundary edge uses already certify both
                        // parameter-space endpoints against these shared
                        // vertices. A planar surface and both authored lines
                        // are affine, so those endpoint certificates prove
                        // the complete chord image.
                        true,
                    )
                }
            };
        let edge_domain = curve_geometry.domain().clone();
        let reverse_pcurve = forward_pcurve.reversed()?;
        let reverse_correspondence = forward_correspondence.reversed_pcurve(&forward_pcurve)?;

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        let curve = Curve3Id::from_index(data.curves.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
        data.curves.push(curve_geometry);
        let edge = EdgeId::from_index(data.edges.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
        data.edges.push(Edge {
            start,
            end,
            curve,
            domain: (edge_domain).into(),
            endpoint_certificate: authored_curve,
        });

        let first_pcurve = PcurveId::from_index(data.pcurves.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
        data.pcurves.push(reverse_pcurve);
        let second_pcurve = PcurveId::from_index(data.pcurves.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
        data.pcurves.push(forward_pcurve);

        let first_use = EdgeUseId::from_index(data.edge_uses.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
        data.edge_uses.push(EdgeUse {
            edge,
            direction: Direction::Reversed,
            pcurve: first_pcurve,
            parameter_correspondence: reverse_correspondence,
            image_certificate,
        });
        let second_use = EdgeUseId::from_index(data.edge_uses.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
        data.edge_uses.push(EdgeUse {
            edge,
            direction: Direction::Forward,
            pcurve: second_pcurve,
            parameter_correspondence: forward_correspondence,
            image_certificate,
        });
        first_path.push(first_use);
        second_path.push(second_use);
        data.wires[outer.index()].edge_uses = first_path;
        data.wires[outer.index()].simplicity_certificate = true;
        let second_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: second_path,
            simplicity_certificate: true,
        });

        data.faces[face_id.index()] = Face {
            surface: face.surface,
            orientation: face.orientation,
            boundary: FaceBoundary::Trimmed {
                outer,
                inner: Vec::new(),
            },
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

        reset_model_caches(data);
        let mut first_inner = Vec::new();
        let mut second_inner = Vec::new();
        if !inner.is_empty() {
            let first_path = staged.build_model_wire_curve_path(outer)?;
            let second_path = staged.build_model_wire_curve_path(second_wire)?;
            for wire in inner {
                let contour = staged.build_model_wire_contour(wire)?;
                let representative = contour.segments()[0].start();
                let classify = |path: &CurvePath2| -> Result<bool, TopologyEditError> {
                    match crate::error::classified(
                        path.classify_point(&representative.clone().into()),
                    )
                    .map_err(GeometryError::from)?
                    {
                        Classification::Decided(ContourPointLocation::Inside) => Ok(true),
                        Classification::Decided(ContourPointLocation::Outside) => Ok(false),
                        Classification::Decided(ContourPointLocation::Boundary) => {
                            Err(BuildError::IntersectingFaceWires {
                                first: outer,
                                second: wire,
                            }
                            .into())
                        }
                        Classification::Uncertain(reason) => {
                            Err(GeometryError::PlanarClassificationUnresolved(reason).into())
                        }
                    }
                };
                let in_first = classify(&first_path)?;
                let in_second = classify(&second_path)?;
                match (in_first, in_second) {
                    (true, false) => first_inner.push(wire),
                    (false, true) => second_inner.push(wire),
                    _ => {
                        return Err(BuildError::IntersectingFaceWires {
                            first: outer,
                            second: wire,
                        }
                        .into());
                    }
                }
            }
        }
        let data = Arc::make_mut(&mut staged.data);
        data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
            outer,
            inner: first_inner,
        };
        data.faces[second_face.index()].boundary = FaceBoundary::Trimmed {
            outer: second_wire,
            inner: second_inner,
        };
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
            ArenaFaceSplit {
                edge,
                edge_uses: [first_use, second_use],
                first_wire: outer,
                second_wire,
                first_face: face_id,
                second_face,
            },
        ))
    }
}
