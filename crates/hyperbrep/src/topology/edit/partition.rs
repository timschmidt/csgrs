//! Exact model edit partition operations.

use crate::topology::*;

impl ArenaModel {
    /// Deterministically partitions one face by retained exact
    /// surface-intersection curves.
    ///
    /// Curves use their retained exact data as a canonical ordering key when
    /// that comparison is certifiable. Otherwise their stable authored order
    /// and direction are retained; ordering is not a geometric predicate.
    /// After each split, both endpoints of every remaining curve must belong to
    /// exactly one current descendant. The selected operand pcurve is
    /// materialized without inverse fitting, and every new edge remains
    /// identity-shared by its descendants.
    pub fn split_face_by_surface_curves(
        &self,
        face_id: FaceId,
        curves: &[SurfaceIntersectionCurve],
        operand: SurfaceIntersectionOperand,
    ) -> Result<(Self, ArenaFacePartition), TopologyEditError> {
        crate::context::strict(|| {
            self.face(face_id)
                .ok_or(TopologyEditError::InvalidReference {
                    kind: EntityKind::Face,
                    index: face_id.index(),
                })?;
            let mut boundary_traces = Vec::new();
            let mut ordered = Vec::with_capacity(curves.len());
            for (source_index, intersection) in curves.iter().enumerate() {
                if let Some(boundary) =
                    self.retained_surface_trace_on_boundary(face_id, intersection, operand)?
                {
                    boundary_traces.push(ArenaFaceTracePartition {
                        source_index,
                        segments: vec![intersection.curve().clone()],
                        splits: Vec::new(),
                        boundary: Some(boundary),
                        snapshots: Vec::new(),
                    });
                    continue;
                }
                let reversed = intersection.reversed()?;
                let intersection = match compare_curve3_exact_data(
                    &intersection.curve().exact_data(),
                    &reversed.curve().exact_data(),
                ) {
                    Ok(std::cmp::Ordering::Greater) => reversed,
                    Ok(_) | Err(GeometryError::PredicateUnresolved { .. }) => intersection.clone(),
                    Err(error) => return Err(error.into()),
                };
                let trace = OrderedSurfaceCurveTrace {
                    source_index,
                    exact_key: intersection.curve().clone(),
                    intersection,
                };
                let mut insertion = ordered.len();
                while insertion > 0
                    && compare_ordered_surface_curve_traces(&trace, &ordered[insertion - 1])?
                        == std::cmp::Ordering::Less
                {
                    insertion -= 1;
                }
                if insertion > 0
                    && compare_ordered_surface_curve_traces(&trace, &ordered[insertion - 1])?
                        == std::cmp::Ordering::Equal
                {
                    return Err(TopologyEditError::DuplicateFaceSplitTrace {
                        first: ordered[insertion - 1].source_index,
                        second: source_index,
                    });
                }
                if insertion < ordered.len()
                    && compare_ordered_surface_curve_traces(&trace, &ordered[insertion])?
                        == std::cmp::Ordering::Equal
                {
                    return Err(TopologyEditError::DuplicateFaceSplitTrace {
                        first: ordered[insertion].source_index,
                        second: source_index,
                    });
                }
                ordered.insert(insertion, trace);
            }
            let mut split_parameters = ordered
                .iter()
                .map(|trace| {
                    vec![
                        trace.intersection.curve().domain().start().clone(),
                        trace.intersection.curve().domain().end().clone(),
                    ]
                })
                .collect::<Vec<_>>();
            let materialized = ordered
                .iter()
                .map(|trace| trace.intersection.pcurve(operand).materialize())
                .collect::<Result<Vec<_>, _>>()?;
            for first_index in 0..ordered.len() {
                for second_index in (first_index + 1)..ordered.len() {
                    let relation = materialized[first_index]
                        .curve()
                        .intersect_curve(materialized[second_index].curve())
                        .map_err(GeometryError::from)?;
                    if !relation.is_complete() {
                        return Err(GeometryError::UnsupportedIntersection.into());
                    }
                    if !relation.overlaps().is_empty() {
                        return Err(TopologyEditError::OverlappingFaceSplitTraces {
                            first: ordered[first_index].source_index,
                            second: ordered[second_index].source_index,
                        });
                    }
                    for contact in relation.contacts() {
                        let first_parameter = resolve_planar_classification(
                            crate::error::classified(contact.first().parameter())
                                .map_err(GeometryError::from)?,
                        )?;
                        let second_parameter = resolve_planar_classification(
                            crate::error::classified(contact.second().parameter())
                                .map_err(GeometryError::from)?,
                        )?;
                        let first_parameter = first_parameter
                            .scalar()
                            .ok_or(GeometryError::UnrepresentableParameter)?;
                        let second_parameter = second_parameter
                            .scalar()
                            .ok_or(GeometryError::UnrepresentableParameter)?;
                        let first_spatial =
                            materialized[first_index].spatial_parameter_at(first_parameter)?;
                        if !ordered[first_index]
                            .intersection
                            .curve()
                            .domain()
                            .contains(&first_spatial)?
                        {
                            return Err(GeometryError::UnsupportedIntersection.into());
                        }
                        insert_exact_split_parameter(
                            &mut split_parameters[second_index],
                            materialized[second_index].spatial_parameter_at(second_parameter)?,
                        )?;
                    }
                }
            }
            if ordered.len() >= 2
                && split_parameters
                    .iter()
                    .all(|parameters| parameters.len() == 2)
            {
                let bridges = ordered
                    .iter()
                    .map(|trace| (trace.source_index, &trace.intersection))
                    .collect::<Vec<_>>();
                if let Some((model, mut partition)) =
                    self.split_face_by_surface_bridge_cycle(face_id, &bridges, operand)?
                {
                    partition.traces.extend(boundary_traces);
                    return Ok((model, partition));
                }
            }
            let mut staged = self.clone();
            let mut faces = vec![face_id];
            let mut traces = Vec::with_capacity(ordered.len());
            for (trace, parameters) in ordered.into_iter().zip(split_parameters) {
                let segments = parameters
                    .windows(2)
                    .map(|range| trace.intersection.subcurve(&range[0], &range[1]))
                    .collect::<Result<Vec<_>, _>>()?;
                let mut splits = Vec::with_capacity(segments.len());
                let mut snapshots = Vec::with_capacity(segments.len());
                let whole_closed_trace = segments.len() == 1
                    && matches!(
                        point3_equal(
                            &segments[0].curve().start()?,
                            &segments[0].curve().end()?,
                            crate::STRICT_PREDICATES,
                        ),
                        PredicateOutcome::Decided { value: true, .. }
                    );
                if whole_closed_trace {
                    let segment = &segments[0];
                    let mut candidates = Vec::new();
                    for (index, candidate) in faces.iter().copied().enumerate() {
                        match staged.split_face_by_surface_curve(candidate, segment, operand) {
                            Ok(result) => candidates.push((index, result)),
                            Err(TopologyEditError::ClosedFaceSplitNotInMaterial { .. }) => {}
                            Err(error) => return Err(error),
                        }
                    }
                    let (face_index, (next, split)) = match candidates.as_slice() {
                        [(index, result)] => (*index, result.clone()),
                        [] => {
                            return Err(TopologyEditError::FaceSplitTraceNotInSingleRegion {
                                face: face_id,
                                trace: trace.source_index,
                                segment: 0,
                            });
                        }
                        _ => {
                            return Err(TopologyEditError::FaceSplitTraceAmbiguous {
                                face: face_id,
                                trace: trace.source_index,
                                segment: 0,
                            });
                        }
                    };
                    snapshots.push((staged.clone(), next.clone()));
                    staged = next;
                    faces[face_index] = split.first_face();
                    faces.insert(face_index + 1, split.second_face());
                    splits.push(split);
                }
                for (segment_index, segment) in segments.iter().enumerate() {
                    if whole_closed_trace {
                        break;
                    }
                    let curve = segment.curve();
                    let start = curve.start()?;
                    let end = curve.end()?;
                    let mut candidates = Vec::new();
                    for (index, candidate) in faces.iter().copied().enumerate() {
                        let start_location = staged.locate_surface_curve_split_endpoint(
                            candidate,
                            Endpoint::Start,
                            &start,
                            segment.start_boundary_contacts(operand),
                        );
                        let end_location = staged.locate_surface_curve_split_endpoint(
                            candidate,
                            Endpoint::End,
                            &end,
                            segment.end_boundary_contacts(operand),
                        );
                        match (start_location, end_location) {
                            (Ok(_), Ok(_)) => candidates.push((index, candidate)),
                            (
                                Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary {
                                    ..
                                }),
                                _,
                            )
                            | (
                                _,
                                Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary {
                                    ..
                                }),
                            ) => {}
                            (Err(error), _) | (_, Err(error)) => return Err(error),
                        }
                    }
                    if candidates.len() > 1 {
                        let midpoint = ((curve.domain().start() + curve.domain().end())
                            / Real::from(2))
                        .map_err(GeometryError::Numeric)?;
                        let parameter = segment.pcurve(operand).point_at(&midpoint)?;
                        let witness = CurvePoint2::new(parameter.x, parameter.y);
                        let mut interior_candidates = Vec::with_capacity(candidates.len());
                        for (index, candidate) in candidates {
                            match staged.classify_surface_parameter_on_face(candidate, &witness)? {
                                Classification::Decided(ContourPointLocation::Inside) => {
                                    interior_candidates.push((index, candidate));
                                }
                                Classification::Decided(ContourPointLocation::Outside) => {}
                                Classification::Decided(ContourPointLocation::Boundary) => {
                                    return Err(TopologyEditError::FaceSplitTraceAmbiguous {
                                        face: face_id,
                                        trace: trace.source_index,
                                        segment: segment_index,
                                    });
                                }
                                Classification::Uncertain(reason) => {
                                    return Err(GeometryError::PlanarClassificationUnresolved(
                                        reason,
                                    )
                                    .into());
                                }
                            }
                        }
                        candidates = interior_candidates;
                    }
                    let (face_index, candidate) = match candidates.as_slice() {
                        [(index, candidate)] => (*index, *candidate),
                        [] => {
                            return Err(TopologyEditError::FaceSplitTraceNotInSingleRegion {
                                face: face_id,
                                trace: trace.source_index,
                                segment: segment_index,
                            });
                        }
                        _ => {
                            return Err(TopologyEditError::FaceSplitTraceAmbiguous {
                                face: face_id,
                                trace: trace.source_index,
                                segment: segment_index,
                            });
                        }
                    };
                    let (next, split) =
                        staged.split_face_by_surface_curve(candidate, segment, operand)?;
                    snapshots.push((staged.clone(), next.clone()));
                    staged = next;
                    faces[face_index] = split.first_face();
                    faces.insert(face_index + 1, split.second_face());
                    splits.push(split);
                }
                traces.push(ArenaFaceTracePartition {
                    source_index: trace.source_index,
                    segments: segments
                        .into_iter()
                        .map(|segment| segment.curve().clone())
                        .collect(),
                    splits,
                    boundary: None,
                    snapshots,
                });
            }
            traces.extend(boundary_traces);
            Ok((
                staged,
                ArenaFacePartition {
                    source_face: face_id,
                    faces,
                    traces,
                },
            ))
        })
    }

    /// Deterministically partitions one planar face by exact line fragments.
    ///
    /// Trace direction and caller order do not affect the resulting topology:
    /// unordered endpoint pairs are sorted by certified lexicographic point
    /// order. Every later trace is subdivided at exact crossings with earlier
    /// traces, then every arranged segment must attach to exactly one current
    /// descendant face. Duplicates, positive-length overlaps, and ambiguous
    /// descendant attachment are typed errors.
    pub fn split_face_by_curves(
        &self,
        face_id: FaceId,
        curves: &[Curve3],
    ) -> Result<(Self, ArenaFacePartition), TopologyEditError> {
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

            let mut traces = Vec::with_capacity(curves.len());
            for (source_index, curve) in curves.iter().enumerate() {
                if curve.kind() != Curve3Kind::Line {
                    return Err(TopologyEditError::UnsupportedFaceSplitCurve(curve.kind()));
                }
                let start = curve.start()?;
                let end = curve.end()?;
                let (lower, upper) =
                    if exact_point_order(&start, &end)? == std::cmp::Ordering::Greater {
                        (end, start)
                    } else {
                        (start, end)
                    };
                let trace = OrderedFaceSplitTrace {
                    source_index,
                    curve: Curve3::line(lower.clone(), upper.clone())?,
                    lower,
                    upper,
                };
                let mut insertion = traces.len();
                while insertion > 0
                    && compare_ordered_face_split_traces(&trace, &traces[insertion - 1])?
                        == std::cmp::Ordering::Less
                {
                    insertion -= 1;
                }
                if insertion > 0
                    && compare_ordered_face_split_traces(&trace, &traces[insertion - 1])?
                        == std::cmp::Ordering::Equal
                {
                    return Err(TopologyEditError::DuplicateFaceSplitTrace {
                        first: traces[insertion - 1].source_index,
                        second: source_index,
                    });
                }
                if insertion < traces.len()
                    && compare_ordered_face_split_traces(&trace, &traces[insertion])?
                        == std::cmp::Ordering::Equal
                {
                    return Err(TopologyEditError::DuplicateFaceSplitTrace {
                        first: traces[insertion].source_index,
                        second: source_index,
                    });
                }
                traces.insert(insertion, trace);
            }

            let mut staged = self.clone();
            let mut faces = vec![face_id];
            let mut prior_lines = Vec::with_capacity(traces.len());
            let mut partitioned_traces = Vec::with_capacity(traces.len());
            for trace in traces {
                let planar = projected_face_split_line(surface, &trace)?;
                let segments = arranged_face_split_segments(&trace, &planar, &prior_lines)?;
                let mut trace_splits = Vec::with_capacity(segments.len());
                let mut snapshots = Vec::with_capacity(segments.len());
                for (segment_index, segment) in segments.iter().enumerate() {
                    let start = segment.start()?;
                    let end = segment.end()?;
                    let mut candidates = Vec::new();
                    for (index, candidate) in faces.iter().copied().enumerate() {
                        let start_location =
                            staged.locate_face_split_endpoint(candidate, Endpoint::Start, &start);
                        let end_location =
                            staged.locate_face_split_endpoint(candidate, Endpoint::End, &end);
                        match (start_location, end_location) {
                            (Ok(_), Ok(_)) => candidates.push((index, candidate)),
                            (
                                Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary {
                                    ..
                                }),
                                _,
                            )
                            | (
                                _,
                                Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary {
                                    ..
                                }),
                            ) => {}
                            (Err(error), _) | (_, Err(error)) => return Err(error),
                        }
                    }
                    let (face_index, candidate) = match candidates.as_slice() {
                        [(index, candidate)] => (*index, *candidate),
                        [] => {
                            return Err(TopologyEditError::FaceSplitTraceNotInSingleRegion {
                                face: face_id,
                                trace: trace.source_index,
                                segment: segment_index,
                            });
                        }
                        _ => {
                            return Err(TopologyEditError::FaceSplitTraceAmbiguous {
                                face: face_id,
                                trace: trace.source_index,
                                segment: segment_index,
                            });
                        }
                    };
                    let (next, split) = staged.split_face_by_curve(candidate, segment)?;
                    snapshots.push((staged.clone(), next.clone()));
                    staged = next;
                    faces[face_index] = split.face.first_face;
                    faces.insert(face_index + 1, split.face.second_face);
                    trace_splits.push(ArenaSurfaceCurveFaceSplit::Open(split));
                }
                prior_lines.push((trace.source_index, planar));
                partitioned_traces.push(ArenaFaceTracePartition {
                    source_index: trace.source_index,
                    segments,
                    splits: trace_splits,
                    boundary: None,
                    snapshots,
                });
            }
            Ok((
                staged,
                ArenaFacePartition {
                    source_face: face_id,
                    faces,
                    traces: partitioned_traces,
                },
            ))
        })
    }

    pub(in crate::topology) fn split_face_by_surface_bridge_cycle(
        &self,
        face_id: FaceId,
        bridges: &[(usize, &SurfaceIntersectionCurve)],
        operand: SurfaceIntersectionOperand,
    ) -> Result<Option<(Self, ArenaFacePartition)>, TopologyEditError> {
        let face = self.face(face_id).expect("surface split prevalidates face");
        let FaceBoundary::Trimmed {
            outer,
            inner: inner_wires,
        } = &face.boundary
        else {
            return Ok(None);
        };
        if inner_wires.is_empty() {
            return Ok(None);
        }

        let mut endpoint_wires = Vec::with_capacity(bridges.len());
        for (_, bridge) in bridges {
            let start = bridge.curve().start()?;
            let end = bridge.curve().end()?;
            let locate = |endpoint, point| match self
                .locate_face_boundary_split_endpoint(face_id, endpoint, point)
            {
                Ok(location) => Ok(Some(location)),
                Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary { .. }) => Ok(None),
                Err(error) => Err(error),
            };
            let Some((start_wire, _)) = locate(Endpoint::Start, &start)? else {
                return Ok(None);
            };
            let Some((end_wire, _)) = locate(Endpoint::End, &end)? else {
                return Ok(None);
            };
            if start_wire == end_wire {
                return Ok(None);
            }
            endpoint_wires.push((start_wire, end_wire));
        }
        let mut wire_incidence = std::collections::HashMap::<WireId, Vec<usize>>::new();
        for (bridge_index, (start_wire, end_wire)) in endpoint_wires.iter().enumerate() {
            wire_incidence
                .entry(*start_wire)
                .or_default()
                .push(bridge_index);
            wire_incidence
                .entry(*end_wire)
                .or_default()
                .push(bridge_index);
        }
        if !wire_incidence.contains_key(outer)
            || wire_incidence
                .values()
                .any(|incidence| incidence.len() != 2)
            || wire_incidence
                .keys()
                .any(|wire| *wire != *outer && !inner_wires.contains(wire))
            || wire_incidence.len() != bridges.len()
        {
            return Ok(None);
        }
        let mut visited_wires = std::collections::HashSet::new();
        let mut frontier = vec![*outer];
        while let Some(wire) = frontier.pop() {
            if !visited_wires.insert(wire) {
                continue;
            }
            for bridge_index in &wire_incidence[&wire] {
                let (start_wire, end_wire) = endpoint_wires[*bridge_index];
                frontier.push(if start_wire == wire {
                    end_wire
                } else {
                    start_wire
                });
            }
        }
        if visited_wires.len() != wire_incidence.len() {
            return Ok(None);
        }
        let target_inner = wire_incidence
            .keys()
            .copied()
            .filter(|wire| *wire != *outer)
            .min_by_key(|wire| wire.index())
            .expect("a bridge cycle through the outer wire contains an inner wire");

        struct AttachedBridge {
            source_index: usize,
            intersection: SurfaceIntersectionCurve,
            start_vertex: VertexId,
            end_vertex: VertexId,
            start_edge: Option<ArenaEdgeSplit>,
            end_edge: Option<ArenaEdgeSplit>,
            start_wire: WireId,
            end_wire: WireId,
        }

        let mut staged = self.clone();
        let mut attached = Vec::with_capacity(bridges.len());
        for (source_index, bridge) in bridges {
            let start = bridge.curve().start()?;
            let end = bridge.curve().end()?;
            let (next, start_wire, start_vertex, start_edge) =
                staged.attach_face_boundary_split_endpoint(face_id, Endpoint::Start, &start)?;
            staged = next;
            let (next, end_wire, end_vertex, end_edge) =
                staged.attach_face_boundary_split_endpoint(face_id, Endpoint::End, &end)?;
            staged = next;
            attached.push(AttachedBridge {
                source_index: *source_index,
                intersection: (*bridge).clone(),
                start_vertex,
                end_vertex,
                start_edge,
                end_edge,
                start_wire,
                end_wire,
            });
        }

        let mut attached_incidence =
            std::collections::HashMap::<WireId, Vec<(usize, VertexId)>>::new();
        for (bridge_index, bridge) in attached.iter().enumerate() {
            attached_incidence
                .entry(bridge.start_wire)
                .or_default()
                .push((bridge_index, bridge.start_vertex));
            attached_incidence
                .entry(bridge.end_wire)
                .or_default()
                .push((bridge_index, bridge.end_vertex));
        }
        if attached_incidence
            .values()
            .any(|incidence| incidence.len() != 2 || incidence[0].1 == incidence[1].1)
        {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }

        let directed_start = |model: &ArenaModel, edge_use_id: EdgeUseId| {
            let edge_use = model.edge_use(edge_use_id).expect("validated edge use");
            let edge = model.edge(edge_use.edge).expect("validated edge");
            match edge_use.direction {
                Direction::Forward => edge.start,
                Direction::Reversed => edge.end,
            }
        };
        let wire_uses = |model: &ArenaModel, wire: WireId| {
            model
                .wire(wire)
                .expect("validated bridge boundary wire")
                .edge_uses
                .clone()
        };
        let unique_index = |model: &ArenaModel,
                            uses: &[EdgeUseId],
                            vertex: VertexId|
         -> Result<usize, TopologyEditError> {
            let matches = uses
                .iter()
                .enumerate()
                .filter_map(|(index, edge_use)| {
                    (directed_start(model, *edge_use) == vertex).then_some(index)
                })
                .collect::<Vec<_>>();
            match matches.as_slice() {
                [index] => Ok(*index),
                _ => Err(TopologyEditError::VertexNotOnOuterBoundary {
                    face: face_id,
                    vertex,
                }),
            }
        };
        let cyclic_path = |uses: &[EdgeUseId], from: usize, to: usize| {
            let mut path = Vec::new();
            let mut index = from;
            while index != to {
                path.push(uses[index]);
                index = (index + 1) % uses.len();
            }
            path
        };
        let mut boundary_uses = std::collections::HashMap::<WireId, Vec<EdgeUseId>>::new();
        let mut boundary_indices = std::collections::HashMap::<(WireId, usize), usize>::new();
        for (wire, incidence) in &attached_incidence {
            let uses = wire_uses(&staged, *wire);
            for (bridge_index, vertex) in incidence {
                boundary_indices.insert(
                    (*wire, *bridge_index),
                    unique_index(&staged, &uses, *vertex)?,
                );
            }
            boundary_uses.insert(*wire, uses);
        }

        struct AuthoredBridge {
            curve: Curve3,
            pcurve: Pcurve,
            correspondence: ParameterCorrespondence,
            image_certificate: bool,
            start_vertex: VertexId,
            end_vertex: VertexId,
        }
        let authored = attached
            .iter()
            .map(|bridge| {
                let materialized = bridge.intersection.pcurve(operand).materialize()?;
                let correspondence = match materialized.correspondence() {
                    SurfacePcurveCorrespondence::Affine { scale, offset } => {
                        ParameterCorrespondence::affine(scale.clone(), offset.clone())?
                    }
                    SurfacePcurveCorrespondence::AngularSweep { .. } => {
                        return Err(TopologyEditError::UnsupportedFaceSplitCurve(
                            bridge.intersection.curve().kind(),
                        ));
                    }
                };
                Ok(AuthoredBridge {
                    curve: bridge.intersection.curve().clone(),
                    pcurve: Pcurve::new(materialized.curve().clone()),
                    correspondence,
                    image_certificate: bridge.intersection.has_image_certificate(operand),
                    start_vertex: bridge.start_vertex,
                    end_vertex: bridge.end_vertex,
                })
            })
            .collect::<Result<Vec<_>, TopologyEditError>>()?;

        let mut staged = staged;
        let data = Arc::make_mut(&mut staged.data);
        let mut bridge_edges = Vec::with_capacity(bridges.len());
        let mut bridge_forward_uses = Vec::with_capacity(bridges.len());
        let mut bridge_reverse_uses = Vec::with_capacity(bridges.len());
        for bridge in &authored {
            let curve_id = Curve3Id::from_index(data.curves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
            data.curves.push(bridge.curve.clone());
            let edge = EdgeId::from_index(data.edges.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
            data.edges.push(Edge {
                start: bridge.start_vertex,
                end: bridge.end_vertex,
                curve: curve_id,
                domain: (bridge.curve.domain().clone()).into(),
                endpoint_certificate: true,
            });
            let forward_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(bridge.pcurve.clone());
            let reverse_pcurve_geometry = bridge.pcurve.reversed()?;
            let reverse_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(reverse_pcurve_geometry.clone());
            let (scale, offset) = bridge
                .correspondence
                .affine_coefficients()
                .expect("boundary bridge requires affine correspondence");
            let reverse_correspondence = ParameterCorrespondence::affine(
                -scale.clone(),
                scale * (bridge.pcurve.represented_start()? + bridge.pcurve.represented_end()?)
                    + offset,
            )?;
            let forward_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Forward,
                pcurve: forward_pcurve,
                parameter_correspondence: bridge.correspondence.clone(),
                image_certificate: bridge.image_certificate,
            });
            let reverse_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Reversed,
                pcurve: reverse_pcurve,
                parameter_correspondence: reverse_correspondence,
                image_certificate: bridge.image_certificate,
            });
            bridge_edges.push(edge);
            bridge_forward_uses.push(forward_use);
            bridge_reverse_uses.push(reverse_use);
        }

        let bridge_use = |index: usize, from: VertexId, to: VertexId| {
            if authored[index].start_vertex == from && authored[index].end_vertex == to {
                bridge_forward_uses[index]
            } else {
                debug_assert!(
                    authored[index].start_vertex == to && authored[index].end_vertex == from
                );
                bridge_reverse_uses[index]
            }
        };
        let first_bridge = &attached[0];
        let first_from_start = first_bridge.start_wire.index() < first_bridge.end_wire.index();
        let build_path =
            |from_start: bool| -> Result<(Vec<EdgeUseId>, Vec<EdgeUseId>), TopologyEditError> {
                let (start_wire, start_vertex, mut current_wire, first_end_vertex) = if from_start {
                    (
                        first_bridge.start_wire,
                        first_bridge.start_vertex,
                        first_bridge.end_wire,
                        first_bridge.end_vertex,
                    )
                } else {
                    (
                        first_bridge.end_wire,
                        first_bridge.end_vertex,
                        first_bridge.start_wire,
                        first_bridge.start_vertex,
                    )
                };
                let mut path = vec![bridge_use(0, start_vertex, first_end_vertex)];
                let mut bridge_uses = vec![None; attached.len()];
                bridge_uses[0] = Some(path[0]);
                let mut current_bridge = 0;
                let mut visited = std::collections::HashSet::from([0]);
                loop {
                    let incidence = &attached_incidence[&current_wire];
                    let (next_bridge, next_vertex) = incidence
                        .iter()
                        .copied()
                        .find(|(bridge_index, _)| *bridge_index != current_bridge)
                        .expect("validated bridge-cycle degree");
                    let uses = &boundary_uses[&current_wire];
                    path.extend(cyclic_path(
                        uses,
                        boundary_indices[&(current_wire, current_bridge)],
                        boundary_indices[&(current_wire, next_bridge)],
                    ));
                    if next_bridge == 0 {
                        if current_wire != start_wire || next_vertex != start_vertex {
                            return Err(TopologyEditError::DegenerateFaceSplit);
                        }
                        break;
                    }
                    if !visited.insert(next_bridge) {
                        return Err(TopologyEditError::DegenerateFaceSplit);
                    }
                    let bridge = &attached[next_bridge];
                    let (next_wire, opposite_vertex) = if bridge.start_wire == current_wire
                        && bridge.start_vertex == next_vertex
                    {
                        (bridge.end_wire, bridge.end_vertex)
                    } else if bridge.end_wire == current_wire && bridge.end_vertex == next_vertex {
                        (bridge.start_wire, bridge.start_vertex)
                    } else {
                        return Err(TopologyEditError::DegenerateFaceSplit);
                    };
                    let use_id = bridge_use(next_bridge, next_vertex, opposite_vertex);
                    path.push(use_id);
                    bridge_uses[next_bridge] = Some(use_id);
                    current_bridge = next_bridge;
                    current_wire = next_wire;
                }
                if visited.len() != attached.len() {
                    return Err(TopologyEditError::DegenerateFaceSplit);
                }
                Ok((
                    path,
                    bridge_uses
                        .into_iter()
                        .map(|edge_use| edge_use.expect("visited bridge has one use"))
                        .collect(),
                ))
            };
        let (first_path, first_face_bridge_uses) = build_path(first_from_start)?;
        let (second_path, second_face_bridge_uses) = build_path(!first_from_start)?;

        data.wires[outer.index()].edge_uses = first_path;
        data.wires[outer.index()].simplicity_certificate = true;
        data.wires[target_inner.index()].edge_uses = second_path;
        data.faces[face_id.index()] = Face {
            surface: face.surface,
            orientation: face.orientation,
            boundary: FaceBoundary::Trimmed {
                outer: *outer,
                inner: Vec::new(),
            },
        };
        let second_face = FaceId::from_index(data.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        data.faces.push(Face {
            surface: face.surface,
            orientation: face.orientation,
            boundary: FaceBoundary::Trimmed {
                outer: target_inner,
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

        let remaining_inner = inner_wires
            .iter()
            .copied()
            .filter(|wire| !attached_incidence.contains_key(wire))
            .collect::<Vec<_>>();
        let first_region = staged.build_model_wire_curve_path(*outer)?;
        let second_region = staged.build_model_wire_curve_path(target_inner)?;
        let mut first_inner = Vec::new();
        let mut second_inner = Vec::new();
        for wire in remaining_inner {
            let contour = staged.build_model_wire_contour(wire)?;
            let representative = contour.segments()[0].start();
            let classify = |path: &CurvePath2| -> Result<bool, TopologyEditError> {
                match crate::error::classified(path.classify_point(&representative.clone().into()))
                    .map_err(GeometryError::from)?
                {
                    Classification::Decided(ContourPointLocation::Inside) => Ok(true),
                    Classification::Decided(ContourPointLocation::Outside) => Ok(false),
                    Classification::Decided(ContourPointLocation::Boundary) => {
                        Err(BuildError::IntersectingFaceWires {
                            first: *outer,
                            second: wire,
                        }
                        .into())
                    }
                    Classification::Uncertain(reason) => {
                        Err(GeometryError::PlanarClassificationUnresolved(reason).into())
                    }
                }
            };
            match (classify(&first_region)?, classify(&second_region)?) {
                (true, false) => first_inner.push(wire),
                (false, true) => second_inner.push(wire),
                _ => {
                    return Err(BuildError::IntersectingFaceWires {
                        first: *outer,
                        second: wire,
                    }
                    .into());
                }
            }
        }
        let data = Arc::make_mut(&mut staged.data);
        data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
            outer: *outer,
            inner: first_inner,
        };
        data.faces[second_face.index()].boundary = FaceBoundary::Trimmed {
            outer: target_inner,
            inner: second_inner,
        };
        let retired_wires = attached_incidence
            .keys()
            .copied()
            .filter(|wire| *wire != *outer && *wire != target_inner)
            .collect::<std::collections::HashSet<_>>();
        let wire_remap = compact_wire_arena(data, &retired_wires)?;
        let first_wire = wire_remap[outer.index()]
            .expect("the source outer wire survives bridge-cycle compaction");
        let second_wire = wire_remap[target_inner.index()]
            .expect("the selected second wire survives bridge-cycle compaction");
        reset_model_caches(data);
        let staged = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;

        let split = ArenaBoundaryBridgeFaceSplit {
            edges: bridge_edges,
            edge_uses: first_face_bridge_uses
                .into_iter()
                .zip(second_face_bridge_uses)
                .map(|(first, second)| [first, second])
                .collect(),
            first_wire,
            second_wire,
            wire_remap,
            first_face: face_id,
            second_face,
        };
        let traces = attached
            .into_iter()
            .enumerate()
            .map(|(bridge_index, bridge)| ArenaFaceTracePartition {
                source_index: bridge.source_index,
                segments: vec![bridge.intersection.curve().clone()],
                splits: vec![ArenaSurfaceCurveFaceSplit::Bridge(
                    ArenaBoundaryBridgeCurveSplit {
                        start_edge: bridge.start_edge,
                        end_edge: bridge.end_edge,
                        bridge_index,
                        face: split.clone(),
                    },
                )],
                boundary: None,
                snapshots: vec![(self.clone(), staged.clone())],
            })
            .collect();
        Ok(Some((
            staged,
            ArenaFacePartition {
                source_face: face_id,
                faces: vec![face_id, second_face],
                traces,
            },
        )))
    }
}
