//! Exact model edit edge operations.

use crate::topology::*;

impl Model {
    /// Splits one canonical edge and every incident edge use at an exact
    /// interior edge parameter.
    ///
    /// The source edge ID is retained for its first canonical half. A vertex,
    /// second edge, split pcurves, and continuation edge uses are appended.
    /// Every owning wire is updated in traversal order, then the entire
    /// snapshot is replayed through [`ModelBuilder`] before publication.
    pub fn split_edge(
        &self,
        edge_id: EdgeId,
        parameter: Real,
    ) -> Result<(Self, EdgeSplit), TopologyEditError> {
        self.split_edge_with_pcurve_parameter(edge_id, parameter, None)
    }

    pub(in crate::topology) fn split_edge_with_pcurve_parameter(
        &self,
        edge_id: EdgeId,
        parameter: Real,
        retained_contact: Option<(EdgeUseId, CurvePoint2, Point3)>,
    ) -> Result<(Self, EdgeSplit), TopologyEditError> {
        let edge = self
            .edge(edge_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Edge,
                index: edge_id.index(),
            })?
            .clone();
        if decided_model_order(compare_reals(
            &parameter,
            edge.domain.start(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(
                &parameter,
                edge.domain.end(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
        {
            return Err(GeometryError::InvalidParameterDomain.into());
        }
        let split_point = if let Some((_, _, point)) = &retained_contact {
            point.clone()
        } else {
            self.curve(edge.curve)
                .expect("validated edge curve ID")
                .point_at(&parameter)?
        };
        let vertex = VertexId::from_index(self.data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        let second = EdgeId::from_index(self.data.edges.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
        let first_domain = ParameterDomain::new(edge.domain.start().clone(), parameter.clone())?;
        let second_domain = ParameterDomain::new(parameter.clone(), edge.domain.end().clone())?;
        let uses = self
            .uses_of_edge(edge_id)
            .expect("validated edge adjacency")
            .to_vec();

        struct UseSplit {
            use_id: EdgeUseId,
            original_pcurve: PcurveId,
            first_pcurve: Pcurve,
            second_pcurve: Pcurve,
            first_edge: EdgeId,
            second_edge: EdgeId,
            direction: Direction,
            first_correspondence: ParameterCorrespondence,
            second_correspondence: ParameterCorrespondence,
        }

        let mut splits = Vec::with_capacity(uses.len());
        for use_id in &uses {
            let edge_use = self
                .edge_use(*use_id)
                .expect("validated edge-use adjacency");
            let pcurve = self
                .pcurve(edge_use.pcurve)
                .expect("validated edge-use pcurve ID");
            let (first_pcurve, second_pcurve) = match &edge_use.parameter_correspondence {
                ParameterCorrespondence::AngularSweep => {
                    if let Some((_, retained_point, _)) = retained_contact
                        .as_ref()
                        .filter(|(retained_use, _, _)| retained_use == use_id)
                    {
                        let arc = pcurve.circular_arc().expect(
                            "validated angular-sweep correspondence uses a circular pcurve",
                        );
                        let (start, end) = match edge_use.direction {
                            Direction::Forward => (edge.domain.start(), edge.domain.end()),
                            Direction::Reversed => (edge.domain.end(), edge.domain.start()),
                        };
                        let fraction = ((&parameter - start) / (end - start))
                            .map_err(|_| GeometryError::ProjectiveDivision)?;
                        let (first, second) = match crate::error::classified(
                            arc.split_at_retained_sweep_point(&fraction, retained_point.clone()),
                        )
                        .map_err(GeometryError::from)?
                        {
                            Classification::Decided(fragments) => fragments,
                            Classification::Uncertain(reason) => {
                                return Err(
                                    GeometryError::PlanarClassificationUnresolved(reason).into()
                                );
                            }
                        };
                        (
                            Pcurve::new(Curve2::from(first)),
                            Pcurve::new(Curve2::from(second)),
                        )
                    } else {
                        let arc = pcurve.circular_arc().expect(
                            "validated angular-sweep correspondence uses a circular pcurve",
                        );
                        let (start, end) = match edge_use.direction {
                            Direction::Forward => (edge.domain.start(), edge.domain.end()),
                            Direction::Reversed => (edge.domain.end(), edge.domain.start()),
                        };
                        let fraction = ((&parameter - start) / (end - start))
                            .map_err(|_| GeometryError::ProjectiveDivision)?;
                        let (first, second) =
                            match crate::error::classified(arc.split_at_sweep_fraction(&fraction))
                                .map_err(GeometryError::from)?
                            {
                                Classification::Decided(fragments) => fragments,
                                Classification::Uncertain(reason) => {
                                    return Err(GeometryError::PlanarClassificationUnresolved(
                                        reason,
                                    )
                                    .into());
                                }
                            };
                        (
                            Pcurve::new(Curve2::from(first)),
                            Pcurve::new(Curve2::from(second)),
                        )
                    }
                }
                ParameterCorrespondence::Affine { .. } => {
                    let pcurve_parameter = edge_use.parameter_correspondence.pcurve_parameter(
                        pcurve,
                        &edge.domain,
                        edge_use.direction,
                        &parameter,
                    )?;
                    pcurve.split_at(&pcurve_parameter)?
                }
            };
            let (first_edge, first_half_domain, second_edge, second_half_domain) =
                match edge_use.direction {
                    Direction::Forward => (edge_id, &first_domain, second, &second_domain),
                    Direction::Reversed => (second, &second_domain, edge_id, &first_domain),
                };
            let first_correspondence = split_parameter_correspondence(
                &edge_use.parameter_correspondence,
                &first_pcurve,
                first_half_domain,
                edge_use.direction,
            )?;
            let second_correspondence = split_parameter_correspondence(
                &edge_use.parameter_correspondence,
                &second_pcurve,
                second_half_domain,
                edge_use.direction,
            )?;
            splits.push(UseSplit {
                use_id: *use_id,
                original_pcurve: edge_use.pcurve,
                first_pcurve,
                second_pcurve,
                first_edge,
                second_edge,
                direction: edge_use.direction,
                first_correspondence,
                second_correspondence,
            });
        }

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        data.vertices.push(Vertex { point: split_point });
        data.edges[edge_id.index()] = Edge {
            start: edge.start,
            end: vertex,
            curve: edge.curve,
            domain: first_domain,
            // The source endpoints were already validated and the new shared
            // endpoint is either evaluated from this curve or retained from
            // certified boundary-contact evidence.
            endpoint_certificate: true,
        };
        data.edges.push(Edge {
            start: vertex,
            end: edge.end,
            curve: edge.curve,
            domain: second_domain,
            endpoint_certificate: true,
        });

        let mut pcurve_use_counts = vec![0_usize; data.pcurves.len()];
        for edge_use in &data.edge_uses {
            pcurve_use_counts[edge_use.pcurve.index()] += 1;
        }
        let mut split_uses = Vec::with_capacity(splits.len());
        for split in splits {
            let first_pcurve_id = if pcurve_use_counts[split.original_pcurve.index()] == 1 {
                data.pcurves[split.original_pcurve.index()] = split.first_pcurve;
                split.original_pcurve
            } else {
                let id = PcurveId::from_index(data.pcurves.len())
                    .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
                data.pcurves.push(split.first_pcurve);
                id
            };
            let second_pcurve_id = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(split.second_pcurve);
            data.edge_uses[split.use_id.index()] = EdgeUse {
                edge: split.first_edge,
                direction: split.direction,
                pcurve: first_pcurve_id,
                parameter_correspondence: split.first_correspondence,
                image_certificate: true,
            };
            let continuation = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge: split.second_edge,
                direction: split.direction,
                pcurve: second_pcurve_id,
                parameter_correspondence: split.second_correspondence,
                image_certificate: true,
            });
            split_uses.push((split.use_id, continuation));
        }
        for wire in &mut data.wires {
            let mut expanded = Vec::with_capacity(wire.edge_uses.len() + split_uses.len());
            for edge_use in &wire.edge_uses {
                expanded.push(*edge_use);
                if let Some((_, continuation)) =
                    split_uses.iter().find(|(source, _)| source == edge_use)
                {
                    expanded.push(*continuation);
                }
            }
            wire.edge_uses = expanded;
        }
        reset_model_caches(data);
        let validated = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;
        Ok((
            validated,
            EdgeSplit {
                vertex,
                first: edge_id,
                second,
                edge_uses: split_uses,
            },
        ))
    }
}
