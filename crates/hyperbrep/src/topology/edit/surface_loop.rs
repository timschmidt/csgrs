//! Exact face splits along closed chains of retained surface curves.

use crate::topology::*;

/// Oriented chain pieces and their oriented end points.
type OrientedSurfaceCurveChain = (Vec<SurfaceIntersectionCurve>, Vec<(Point3, Point3)>);

impl ArenaModel {
    /// Splits a trimmed face along a closed chain of retained surface curves.
    ///
    /// The pieces are taken in the given order; each is used forward or
    /// reversed so that consecutive pieces share exactly equal endpoints and
    /// the last piece ends where the first starts. Every operand pcurve is
    /// materialized without inverse fitting and retains its exact
    /// correspondence. The loop must lie in the face's material: it becomes
    /// a new inner wire of the source face and the outer wire of a new
    /// enclosed face, holes are assigned by exact classification, and the
    /// complete model is revalidated before publication.
    pub fn split_face_by_surface_curve_loop(
        &self,
        face_id: FaceId,
        intersections: &[SurfaceIntersectionCurve],
        operand: SurfaceIntersectionOperand,
    ) -> Result<(Self, ArenaSurfaceCurveLoopFaceSplit), TopologyEditError> {
        crate::context::strict(|| {
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
            if intersections.len() < 2 {
                return Err(TopologyEditError::DegenerateFaceSplit);
            }
            let equal = |a: &Point3, b: &Point3| -> Result<bool, TopologyEditError> {
                Ok(exact_point_order(a, b)? == std::cmp::Ordering::Equal)
            };
            let (oriented, ends) =
                self.orient_surface_curve_chain(face_id, intersections, operand)?;
            if !equal(&ends[ends.len() - 1].1, &ends[0].0)? {
                return Err(TopologyEditError::DegenerateFaceSplit);
            }

            // Exact edge geometry for each oriented piece.
            let mut pieces = Vec::with_capacity(oriented.len());
            for intersection in &oriented {
                let materialized = intersection.pcurve(operand).materialize()?;
                let correspondence = match materialized.correspondence() {
                    SurfacePcurveCorrespondence::Affine { scale, offset } => {
                        ParameterCorrespondence::affine(scale.clone(), offset.clone())?
                    }
                    SurfacePcurveCorrespondence::AngularSweep { .. } => {
                        ParameterCorrespondence::angular_sweep()
                    }
                };
                pieces.push((
                    intersection.curve().clone(),
                    Pcurve::new(materialized.curve().clone()),
                    correspondence,
                    intersection.has_image_certificate(operand),
                ));
            }

            let loop_path = CurvePath2::try_new(
                pieces
                    .iter()
                    .map(|(_, pcurve, _, _)| pcurve.curve().clone())
                    .collect(),
            )
            .map_err(GeometryError::from)?;
            let contour = loop_path.boundary_loop().map_err(GeometryError::from)?;
            let exact_area = crate::error::classified(contour.signed_area())
                .map_err(GeometryError::from)
                .and_then(resolve_planar_classification);
            let area_order = match exact_area {
                Ok(Some(area)) => decided_model_order(compare_reals(
                    &area,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))?,
                // Implicit branches have no exact area integral: a point with
                // nonzero winding certifies the simple loop's orientation.
                _ => winding_orientation(
                    &loop_path,
                    &pieces
                        .iter()
                        .map(|(_, pcurve, _, _)| pcurve.curve().clone())
                        .collect::<Vec<_>>(),
                )?,
            };
            if area_order == std::cmp::Ordering::Equal {
                return Err(TopologyEditError::DegenerateFaceSplit);
            }
            let expected_outer_order = match face.orientation {
                Orientation::Forward => std::cmp::Ordering::Greater,
                Orientation::Reversed => std::cmp::Ordering::Less,
            };
            let interior_forward = area_order == expected_outer_order;

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
            let loop_start = pieces[0].1.curve().start();
            let outer_path = self.build_model_wire_curve_path(*outer)?;
            if classify(&outer_path, &loop_start)? != ContourPointLocation::Inside {
                return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
            }
            let mut enclosed_inner = Vec::new();
            let mut exterior_inner = Vec::new();
            for hole in inner {
                let hole_path = self.build_model_wire_curve_path(*hole)?;
                if classify(&hole_path, &loop_start)? != ContourPointLocation::Outside {
                    return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
                }
                match classify(&loop_path, &hole_path.start())? {
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
            let mut vertices = Vec::with_capacity(pieces.len());
            for (start, _) in &ends {
                let vertex = VertexId::from_index(data.vertices.len())
                    .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
                data.vertices.push(Vertex {
                    point: start.clone().into(),
                });
                vertices.push(vertex);
            }
            let count = pieces.len();
            let mut edges = Vec::with_capacity(count);
            let mut forward_uses = Vec::with_capacity(count);
            let mut reverse_uses = Vec::with_capacity(count);
            for (index, (curve, forward_pcurve, forward_map, image_certificate)) in
                pieces.into_iter().enumerate()
            {
                let reverse_pcurve = forward_pcurve.reversed()?;
                let reverse_map = forward_map.reversed_pcurve(&forward_pcurve)?;
                let curve_id = Curve3Id::from_index(data.curves.len())
                    .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
                let domain = curve.domain().clone();
                data.curves.push(curve);
                let edge = EdgeId::from_index(data.edges.len())
                    .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
                data.edges.push(Edge {
                    start: vertices[index],
                    end: vertices[(index + 1) % count],
                    curve: curve_id,
                    domain,
                    endpoint_certificate: false,
                });
                edges.push(edge);
                for (direction, pcurve, map, uses) in [
                    (
                        Direction::Forward,
                        forward_pcurve,
                        forward_map,
                        &mut forward_uses,
                    ),
                    (
                        Direction::Reversed,
                        reverse_pcurve,
                        reverse_map,
                        &mut reverse_uses,
                    ),
                ] {
                    let pcurve_id = PcurveId::from_index(data.pcurves.len())
                        .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
                    data.pcurves.push(pcurve);
                    let edge_use = EdgeUseId::from_index(data.edge_uses.len())
                        .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
                    data.edge_uses.push(EdgeUse {
                        edge,
                        direction,
                        pcurve: pcurve_id,
                        parameter_correspondence: map,
                        image_certificate,
                    });
                    uses.push(edge_use);
                }
            }
            reverse_uses.reverse();
            let (interior_uses, exterior_uses) = if interior_forward {
                (forward_uses, reverse_uses)
            } else {
                (reverse_uses, forward_uses)
            };
            let first_wire = WireId::from_index(data.wires.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
            data.wires.push(Wire {
                edge_uses: exterior_uses,
                simplicity_certificate: true,
            });
            let second_wire = WireId::from_index(data.wires.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
            data.wires.push(Wire {
                edge_uses: interior_uses,
                simplicity_certificate: true,
            });
            exterior_inner.push(first_wire);
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
                    outer: second_wire,
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
                ArenaSurfaceCurveLoopFaceSplit {
                    vertices,
                    edges,
                    first_wire,
                    second_wire,
                    first_face: face_id,
                    second_face,
                },
            ))
        })
    }

    /// Orients retained surface curves into one chain: each piece is used
    /// forward or reversed so that consecutive pieces share exactly equal
    /// endpoints. Pieces on the face boundary or with equal ends, and pieces
    /// that do not chain, are degenerate. Returns the oriented pieces and
    /// their oriented end points.
    pub(in crate::topology) fn orient_surface_curve_chain(
        &self,
        face_id: FaceId,
        intersections: &[SurfaceIntersectionCurve],
        operand: SurfaceIntersectionOperand,
    ) -> Result<OrientedSurfaceCurveChain, TopologyEditError> {
        let equal = |a: &Point3, b: &Point3| -> Result<bool, TopologyEditError> {
            Ok(exact_point_order(a, b)? == std::cmp::Ordering::Equal)
        };
        let mut oriented: Vec<SurfaceIntersectionCurve> = Vec::with_capacity(intersections.len());
        let mut ends: Vec<(Point3, Point3)> = Vec::with_capacity(intersections.len());
        for intersection in intersections {
            if self
                .retained_surface_trace_on_boundary(face_id, intersection, operand)?
                .is_some()
            {
                return Err(TopologyEditError::DegenerateFaceSplit);
            }
            let start = intersection.curve().start()?;
            let end = intersection.curve().end()?;
            if equal(&start, &end)? {
                return Err(TopologyEditError::DegenerateFaceSplit);
            }
            let forward = match ends.last() {
                None => {
                    // The first piece may need reversal to meet the second.
                    match intersections.get(1) {
                        None => true,
                        Some(next) => {
                            let (next_start, next_end) =
                                (next.curve().start()?, next.curve().end()?);
                            if equal(&end, &next_start)? || equal(&end, &next_end)? {
                                true
                            } else if equal(&start, &next_start)? || equal(&start, &next_end)? {
                                false
                            } else {
                                return Err(TopologyEditError::DegenerateFaceSplit);
                            }
                        }
                    }
                }
                Some((_, previous)) => {
                    if equal(previous, &start)? {
                        true
                    } else if equal(previous, &end)? {
                        false
                    } else {
                        return Err(TopologyEditError::DegenerateFaceSplit);
                    }
                }
            };
            if forward {
                oriented.push(intersection.clone());
                ends.push((start, end));
            } else {
                oriented.push(intersection.reversed()?);
                ends.push((end, start));
            }
        }
        Ok((oriented, ends))
    }

    /// Splits a trimmed face along an open chain of retained surface curves
    /// running from one outer-boundary point to another through new interior
    /// junction vertices. The chain is oriented by exact endpoint equality;
    /// its ends attach to existing boundary vertices or split the unique
    /// containing boundary edges, every operand pcurve is materialized
    /// without fitting, and the complete model is revalidated.
    pub fn split_face_by_surface_curve_chain(
        &self,
        face_id: FaceId,
        intersections: &[SurfaceIntersectionCurve],
        operand: SurfaceIntersectionOperand,
    ) -> Result<(Self, ArenaSurfaceCurveChainFaceSplit), TopologyEditError> {
        crate::context::strict(|| {
            let face = self
                .face(face_id)
                .ok_or(TopologyEditError::InvalidReference {
                    kind: EntityKind::Face,
                    index: face_id.index(),
                })?;
            if face.outer().is_none() {
                return Err(TopologyEditError::WholeSurfaceFace(face_id));
            }
            if intersections.len() < 2 {
                return Err(TopologyEditError::DegenerateFaceSplit);
            }
            let (oriented, ends) =
                self.orient_surface_curve_chain(face_id, intersections, operand)?;
            let (start, end) = (ends[0].0.clone(), ends[ends.len() - 1].1.clone());
            if exact_point_order(&start, &end)? == std::cmp::Ordering::Equal {
                // A closed chain is a loop split, not an open chain.
                return Err(TopologyEditError::DegenerateFaceSplit);
            }
            let mut pieces = Vec::with_capacity(oriented.len());
            for intersection in &oriented {
                let materialized = intersection.pcurve(operand).materialize()?;
                let correspondence = match materialized.correspondence() {
                    SurfacePcurveCorrespondence::Affine { scale, offset } => {
                        ParameterCorrespondence::affine(scale.clone(), offset.clone())?
                    }
                    SurfacePcurveCorrespondence::AngularSweep { .. } => {
                        ParameterCorrespondence::angular_sweep()
                    }
                };
                pieces.push((
                    intersection.curve().clone(),
                    Pcurve::new(materialized.curve().clone()),
                    correspondence,
                    intersection.has_image_certificate(operand),
                ));
            }
            let first = &oriented[0];
            let last = &oriented[oriented.len() - 1];
            let (staged, start_vertex, start_edge) = self.attach_surface_curve_split_endpoint(
                face_id,
                Endpoint::Start,
                &start,
                first.start_boundary_contacts(operand),
            )?;
            let (staged, end_vertex, end_edge) = staged.attach_surface_curve_split_endpoint(
                face_id,
                Endpoint::End,
                &end,
                last.end_boundary_contacts(operand),
            )?;
            let junctions = ends[..ends.len() - 1]
                .iter()
                .map(|(_, junction)| junction.clone())
                .collect();
            let (staged, face, edges) = staged.split_face_with_chain(
                face_id,
                start_vertex,
                end_vertex,
                Some(pieces),
                junctions,
            )?;
            Ok((
                staged,
                ArenaSurfaceCurveChainFaceSplit {
                    start_edge,
                    end_edge,
                    face,
                    edges,
                },
            ))
        })
    }
}

/// The orientation of a simple closed loop from an exact winding number.
/// Dyadic grid points of the loop's UV bounding box are proposals: one the
/// loop classifies as inside casts a horizontal ray to the right, and the
/// exact signed crossings (the sign of `dv` at each transversal crossing)
/// sum to its winding, whose sign is the orientation. Rays through a
/// junction, tangent rays and undecided predicates discard the proposal; a
/// loop without a decided proposal within the search is typed.
fn winding_orientation(
    loop_path: &CurvePath2,
    curves: &[Curve2],
) -> Result<std::cmp::Ordering, TopologyEditError> {
    const MAX_LEVEL: u32 = 6;
    let order =
        |a: &Real, b: &Real| decided_model_order(compare_reals(a, b, crate::STRICT_PREDICATES));
    let mut bounds: Option<[Real; 4]> = None;
    for curve in curves {
        let aabb = curve.bounds().map_err(GeometryError::from)?;
        let next = [
            aabb.min_x().clone(),
            aabb.min_y().clone(),
            aabb.max_x().clone(),
            aabb.max_y().clone(),
        ];
        bounds = Some(match bounds {
            None => next,
            Some(mut current) => {
                for axis in 0..2 {
                    if order(&next[axis], &current[axis])? == std::cmp::Ordering::Less {
                        current[axis] = next[axis].clone();
                    }
                    if order(&next[axis + 2], &current[axis + 2])? == std::cmp::Ordering::Greater {
                        current[axis + 2] = next[axis + 2].clone();
                    }
                }
                current
            }
        });
    }
    let [min_x, min_y, max_x, max_y] = bounds.ok_or(TopologyEditError::DegenerateFaceSplit)?;
    let (width, height) = (&max_x - &min_x, &max_y - &min_y);
    let beyond = &max_x + Real::one();
    // The exact winding of an interior proposal, or `None` to discard it.
    let winding = |x: Real, y: Real| -> Result<Option<i32>, TopologyEditError> {
        let point = hypercurve::Point2::new(x.clone(), y.clone());
        match crate::error::classified(loop_path.classify_point(&point.clone().into())) {
            Ok(Classification::Decided(ContourPointLocation::Inside)) => {}
            _ => return Ok(None),
        }
        let ray = Curve2::from(
            LineSeg2::try_new(point, hypercurve::Point2::new(beyond.clone(), y))
                .map_err(GeometryError::from)?,
        );
        let mut total = 0;
        for curve in curves {
            let Ok(crossing) = curve.intersect_curve(&ray) else {
                return Ok(None);
            };
            if !crossing.blockers().is_empty()
                || !crossing.overlaps().is_empty()
                || !crossing.parameter_components().is_empty()
            {
                return Ok(None);
            }
            let domain = curve.parameter_domain();
            for contact in crossing.contacts() {
                let Ok(parameter) = contact.first().parameter() else {
                    return Ok(None);
                };
                for end in [domain.start(), domain.end()] {
                    if !matches!(
                        Parameter::from_curve(&parameter)?.compare(&Parameter::from_curve(end)?),
                        Ok(std::cmp::Ordering::Less | std::cmp::Ordering::Greater)
                    ) {
                        return Ok(None);
                    }
                }
                let Ok(derivatives) = curve.derivatives_at(&parameter, 1) else {
                    return Ok(None);
                };
                match derivatives[0].coordinate_sign(hypercurve::Axis2::Y) {
                    Ok(hyperreal::RealSign::Positive) => total += 1,
                    Ok(hyperreal::RealSign::Negative) => total -= 1,
                    _ => return Ok(None),
                }
            }
        }
        Ok(Some(total))
    };
    for level in 1..=MAX_LEVEL {
        let cells = 1_i64 << level;
        for i in (1..cells).step_by(2) {
            for j in (1..cells).step_by(2) {
                let fraction =
                    |k: i64| (Real::from(k) / Real::from(cells)).map_err(GeometryError::Numeric);
                match winding(
                    &min_x + &width * fraction(i)?,
                    &min_y + &height * fraction(j)?,
                )? {
                    Some(total) if total > 0 => return Ok(std::cmp::Ordering::Greater),
                    Some(total) if total < 0 => return Ok(std::cmp::Ordering::Less),
                    _ => {}
                }
            }
        }
    }
    Err(GeometryError::UnsupportedPcurveContour.into())
}
