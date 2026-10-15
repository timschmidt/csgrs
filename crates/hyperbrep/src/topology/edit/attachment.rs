//! Exact model edit attachment operations.

use crate::topology::*;

impl ArenaModel {
    pub(in crate::topology) fn attach_surface_curve_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &Point3,
        contacts: &[SurfaceIntersectionBoundaryContact],
    ) -> Result<(Self, VertexId, Option<ArenaEdgeSplit>), TopologyEditError> {
        match self.locate_surface_curve_split_endpoint(face_id, endpoint, point, contacts)? {
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
        start: &Point3,
        end: &Point3,
    ) -> Result<bool, TopologyEditError> {
        let start_contacts = intersection.start_boundary_contacts(operand);
        let end_contacts = intersection.end_boundary_contacts(operand);
        if !start_contacts.is_empty() && !end_contacts.is_empty() {
            let start_location = self.locate_surface_curve_split_endpoint(
                face_id,
                Endpoint::Start,
                start,
                start_contacts,
            )?;
            let end_location = self.locate_surface_curve_split_endpoint(
                face_id,
                Endpoint::End,
                end,
                end_contacts,
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
                    decided_model_order(compare_reals(
                        &start_parameter,
                        &end_parameter,
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Equal
                }
                _ => false,
            });
        }
        Ok(exact_point_order(start, end)? == std::cmp::Ordering::Equal)
    }

    pub(in crate::topology) fn locate_surface_curve_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &Point3,
        contacts: &[SurfaceIntersectionBoundaryContact],
    ) -> Result<FaceSplitEndpointLocation, TopologyEditError> {
        let face = self
            .face(face_id)
            .expect("surface-curve split prevalidates the face");
        let Some(outer) = face.outer() else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        if contacts.is_empty() {
            return self.locate_face_split_endpoint(face_id, endpoint, point);
        }
        let wire = self.wire(outer).expect("validated face outer wire");
        let mut matching_vertices = Vec::new();
        let mut matching_edges = Vec::new();
        for use_id in wire.edge_uses() {
            let edge_use = self.edge_use(*use_id).expect("validated edge use");
            let edge = self.edge(edge_use.edge).expect("validated canonical edge");
            let curve = self.curve(edge.curve).expect("validated edge curve");
            for contact in contacts {
                if !curve.shares_identity(contact.curve()) {
                    continue;
                }
                let start_order = decided_model_order(compare_reals(
                    contact.parameter(),
                    edge.domain.start(),
                    crate::STRICT_PREDICATES,
                ))?;
                if start_order == std::cmp::Ordering::Equal {
                    if !matching_vertices.contains(&edge.start) {
                        matching_vertices.push(edge.start);
                    }
                    continue;
                }
                let end_order = decided_model_order(compare_reals(
                    contact.parameter(),
                    edge.domain.end(),
                    crate::STRICT_PREDICATES,
                ))?;
                if end_order == std::cmp::Ordering::Equal {
                    if !matching_vertices.contains(&edge.end) {
                        matching_vertices.push(edge.end);
                    }
                    continue;
                }
                if start_order == std::cmp::Ordering::Greater
                    && end_order == std::cmp::Ordering::Less
                    && !matching_edges
                        .iter()
                        .any(|(candidate, _, _, _, _)| *candidate == edge_use.edge)
                {
                    matching_edges.push((
                        edge_use.edge,
                        contact.parameter().clone(),
                        *use_id,
                        contact.pcurve_point().clone(),
                        contact.spatial_point().clone(),
                    ));
                }
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
        match matching_edges.as_slice() {
            [(edge, parameter, edge_use, pcurve_point, spatial_point)] => {
                Ok(FaceSplitEndpointLocation::Edge {
                    edge: *edge,
                    parameter: parameter.clone(),
                    pcurve_split: Some(Box::new((
                        *edge_use,
                        pcurve_point.clone(),
                        spatial_point.clone(),
                    ))),
                })
            }
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
