//! Exact model edit attachment operations.

use crate::topology::*;

impl ArenaModel {
    pub(in crate::topology) fn attach_surface_curve_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &GeometricPoint3,
        intersection: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
    ) -> Result<(Self, VertexId, Option<ArenaEdgeSplit>), TopologyEditError> {
        match self.locate_surface_curve_split_endpoint(
            face_id,
            endpoint,
            point,
            intersection,
            operand,
        )? {
            FaceSplitEndpointLocation::Vertex(vertex) => Ok((self.clone(), vertex, None)),
            FaceSplitEndpointLocation::Edge {
                edge,
                parameter,
                pcurve_split,
            } => {
                let (staged, split) = self.split_edge_with_pcurve_parameter(
                    edge,
                    parameter,
                    pcurve_split.map(|witness| *witness),
                )?;
                Ok((staged, split.vertex, Some(split)))
            }
        }
    }

    pub(in crate::topology) fn retained_surface_curve_endpoints_equal(
        &self,
        face_id: FaceId,
        intersection: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
        start: &GeometricPoint3,
        end: &GeometricPoint3,
    ) -> Result<bool, TopologyEditError> {
        let start_contacts = intersection.start_boundary_contacts(operand);
        let end_contacts = intersection.end_boundary_contacts(operand);
        if !start_contacts.is_empty() && !end_contacts.is_empty() {
            let start_location = self.locate_surface_curve_split_endpoint(
                face_id,
                Endpoint::Start,
                start,
                intersection,
                operand,
            )?;
            let end_location = self.locate_surface_curve_split_endpoint(
                face_id,
                Endpoint::End,
                end,
                intersection,
                operand,
            )?;
            return Ok(match (start_location, end_location) {
                (
                    FaceSplitEndpointLocation::Vertex(start),
                    FaceSplitEndpointLocation::Vertex(end),
                ) => start == end,
                (
                    FaceSplitEndpointLocation::Edge {
                        edge: start_edge,
                        parameter: start_parameter,
                        ..
                    },
                    FaceSplitEndpointLocation::Edge {
                        edge: end_edge,
                        parameter: end_parameter,
                        ..
                    },
                ) if start_edge == end_edge => {
                    start_parameter.compare(&end_parameter)? == std::cmp::Ordering::Equal
                }
                _ => false,
            });
        }
        Ok(start.coincides_with(end)?)
    }

    pub(in crate::topology) fn locate_surface_curve_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &GeometricPoint3,
        intersection: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
    ) -> Result<FaceSplitEndpointLocation, TopologyEditError> {
        let face = self
            .face(face_id)
            .expect("surface-curve split prevalidates the face");
        let Some(outer) = face.outer() else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        let contacts = match endpoint {
            Endpoint::Start => intersection.start_boundary_contacts(operand),
            Endpoint::End => intersection.end_boundary_contacts(operand),
        };
        if !contacts.is_empty() {
            match self.locate_retained_contact_endpoint(face_id, endpoint, &[outer], contacts) {
                Ok((_, location)) => return Ok(location),
                Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary { .. }) => {}
                Err(error) => return Err(error),
            }
        } else if let Some(point) = point.try_coordinates() {
            match self.locate_face_split_endpoint(face_id, endpoint, point) {
                Err(TopologyEditError::Geometry(GeometryError::UnrepresentableParameter)) => {}
                result => return result,
            }
        }
        self.locate_surface_pcurve_endpoint(
            face_id,
            endpoint,
            &[outer],
            point,
            intersection,
            operand,
        )
        .map(|(_, location)| location)
    }

    /// Locates a retained contact on material or hole boundaries, preserving
    /// the owning wire and the original canonical curve parameter.
    pub(in crate::topology) fn locate_surface_curve_boundary_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &GeometricPoint3,
        intersection: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
    ) -> Result<(WireId, FaceSplitEndpointLocation), TopologyEditError> {
        let face = self.face(face_id).expect("surface split prevalidates face");
        let wires = face.boundary_wires().copied().collect::<Vec<_>>();
        let contacts = match endpoint {
            Endpoint::Start => intersection.start_boundary_contacts(operand),
            Endpoint::End => intersection.end_boundary_contacts(operand),
        };
        if !contacts.is_empty() {
            match self.locate_retained_contact_endpoint(face_id, endpoint, &wires, contacts) {
                Ok(location) => return Ok(location),
                Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary { .. }) => {}
                Err(error) => return Err(error),
            }
        } else if let Some(point) = point.try_coordinates() {
            match self.locate_face_boundary_split_endpoint(face_id, endpoint, point) {
                Err(TopologyEditError::Geometry(GeometryError::UnrepresentableParameter)) => {}
                result => return result,
            }
        }
        self.locate_surface_pcurve_endpoint(face_id, endpoint, &wires, point, intersection, operand)
    }

    pub(in crate::topology) fn attach_surface_curve_boundary_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &GeometricPoint3,
        intersection: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
    ) -> Result<(Self, WireId, VertexId, Option<ArenaEdgeSplit>), TopologyEditError> {
        let (wire, location) = self.locate_surface_curve_boundary_endpoint(
            face_id,
            endpoint,
            point,
            intersection,
            operand,
        )?;
        match location {
            FaceSplitEndpointLocation::Vertex(vertex) => Ok((self.clone(), wire, vertex, None)),
            FaceSplitEndpointLocation::Edge {
                edge,
                parameter,
                pcurve_split,
            } => {
                let (model, split) = self.split_edge_with_pcurve_parameter(
                    edge,
                    parameter,
                    pcurve_split.map(|contact| *contact),
                )?;
                Ok((model, wire, split.vertex, Some(split)))
            }
        }
    }

    /// Recover missing boundary incidence in the original operand chart. The
    /// planar owner supplies every preimage; the validated edge correspondence
    /// transports each to the canonical spatial parameter. A failed or absent
    /// scalar view is never a proof that the point is outside the boundary.
    fn locate_surface_pcurve_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        wires: &[WireId],
        point: &GeometricPoint3,
        intersection: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
    ) -> Result<(WireId, FaceSplitEndpointLocation), TopologyEditError> {
        let parameter = match endpoint {
            Endpoint::Start => intersection.curve().domain().start(),
            Endpoint::End => intersection.curve().domain().end(),
        };
        let uv = intersection.pcurve(operand).point_at(parameter)?;
        let face = self.face(face_id).expect("surface split prevalidates face");
        let surface = self.surface(face.surface).expect("validated face surface");
        if !surface.point_at(&uv)?.coincides_with(point)? {
            return Err(TopologyEditError::SurfaceCurveEndpointImageMismatch {
                face: face_id,
                endpoint,
            });
        }
        let mut contacts = Vec::new();
        for wire in wires {
            for use_id in self
                .wire(*wire)
                .expect("validated boundary wire")
                .edge_uses()
            {
                let usage = self.edge_use(*use_id).expect("validated edge use");
                let edge = self.edge(usage.edge).expect("validated canonical edge");
                let pcurve = self.pcurve(usage.pcurve).expect("validated pcurve");
                for local in pcurve.parameters_at_point(&uv)? {
                    let parameter = usage.parameter_correspondence.edge_parameter(
                        pcurve,
                        &edge.domain,
                        usage.direction,
                        &local,
                    )?;
                    contacts.push(SurfaceIntersectionBoundaryContact::new(
                        self.curve(edge.curve)
                            .expect("validated edge curve")
                            .clone(),
                        parameter.clone(),
                        Some(parameter),
                        uv.clone(),
                        point.clone(),
                    ));
                }
            }
        }
        self.locate_retained_contact_endpoint(face_id, endpoint, wires, &contacts)
    }

    fn locate_retained_contact_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        wires: &[WireId],
        contacts: &[SurfaceIntersectionBoundaryContact],
    ) -> Result<(WireId, FaceSplitEndpointLocation), TopologyEditError> {
        let mut matching_vertices = Vec::new();
        let mut matching_edges = Vec::new();
        for &wire_id in wires {
            let wire = self.wire(wire_id).expect("validated face boundary wire");
            for use_id in wire.edge_uses() {
                let edge_use = self.edge_use(*use_id).expect("validated edge use");
                let edge = self.edge(edge_use.edge).expect("validated canonical edge");
                let curve = self.curve(edge.curve).expect("validated edge curve");
                for contact in contacts {
                    if !curve.shares_identity(contact.curve()) {
                        continue;
                    }
                    let start_order = contact.parameter().compare(edge.domain.start())?;
                    if start_order == std::cmp::Ordering::Equal {
                        if !matching_vertices.contains(&(wire_id, edge.start)) {
                            matching_vertices.push((wire_id, edge.start));
                        }
                        continue;
                    }
                    let end_order = contact.parameter().compare(edge.domain.end())?;
                    if end_order == std::cmp::Ordering::Equal {
                        if !matching_vertices.contains(&(wire_id, edge.end)) {
                            matching_vertices.push((wire_id, edge.end));
                        }
                        continue;
                    }
                    if start_order == std::cmp::Ordering::Greater
                        && end_order == std::cmp::Ordering::Less
                        && !matching_edges.iter().any(|(wire, candidate, _, _, _)| {
                            *wire == wire_id && *candidate == edge_use.edge
                        })
                    {
                        matching_edges.push((
                            wire_id,
                            edge_use.edge,
                            contact.parameter().clone(),
                            *use_id,
                            contact.clone(),
                        ));
                    }
                }
            }
        }
        match matching_vertices.as_slice() {
            [(wire, vertex)] => return Ok((*wire, FaceSplitEndpointLocation::Vertex(*vertex))),
            [] => {}
            _ => {
                return Err(TopologyEditError::FaceSplitEndpointAmbiguous {
                    face: face_id,
                    endpoint,
                });
            }
        }
        match matching_edges.as_slice() {
            [(wire, edge, parameter, edge_use, contact)] => Ok((
                *wire,
                FaceSplitEndpointLocation::Edge {
                    edge: *edge,
                    parameter: parameter.clone(),
                    pcurve_split: Some(Box::new((*edge_use, contact.clone()))),
                },
            )),
            [] => Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary {
                face: face_id,
                endpoint,
            }),
            _ => Err(TopologyEditError::FaceSplitEndpointAmbiguous {
                face: face_id,
                endpoint,
            }),
        }
    }
}
