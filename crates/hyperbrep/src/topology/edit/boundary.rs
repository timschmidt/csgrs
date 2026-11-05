//! Complete boundary incidence for retained surface traces.

use crate::topology::*;

impl ArenaModel {
    pub(in crate::topology) fn retained_surface_trace_on_boundary(
        &self,
        face_id: FaceId,
        intersection: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
    ) -> Result<Option<ArenaBoundarySurfaceTrace>, TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?;
        let Some(outer) = face.outer() else {
            return Ok(None);
        };
        if !intersection
            .pcurve(operand)
            .materializes_native_boundary_segment()
        {
            return Ok(None);
        }
        let materialized = intersection.pcurve(operand).materialize()?;
        let pcurve = Pcurve::new(materialized.curve().clone());
        let order =
            |a: &Real, b: &Real| decided_model_order(compare_reals(a, b, crate::STRICT_PREDICATES));
        let mut overlaps = Vec::new();
        for wire in std::iter::once(outer).chain(face.inner().iter().copied()) {
            for &edge_use_id in self.wire(wire).expect("validated face wire").edge_uses() {
                let edge_use = self.edge_use(edge_use_id).expect("validated wire use");
                let boundary = self.pcurve(edge_use.pcurve()).expect("validated pcurve");
                if let (Some(trace), Some(boundary)) =
                    (pcurve.line_segment(), boundary.line_segment())
                {
                    match trace
                        .intersect_line(boundary)
                        .map_err(GeometryError::from)?
                    {
                        LineLineIntersection::Overlap {
                            a_range, b_range, ..
                        } => {
                            overlaps.push(ArenaBoundaryTraceOverlap {
                                edge_use: edge_use_id,
                                trace_range: a_range,
                                boundary_range: b_range,
                                native_overlap: None,
                            });
                        }
                        LineLineIntersection::None | LineLineIntersection::Point { .. } => {}
                        // A missing overlap certificate is not evidence of
                        // coincidence. Leave it to the ordinary split validator.
                        LineLineIntersection::Uncertain { .. } => return Ok(None),
                    }
                } else if pcurve.circular_arc().is_some()
                    || pcurve.curve().polynomial_branch().is_some()
                {
                    // Boundary editing may retain a circular image as a
                    // rational Bezier or restriction. Native overlap witnesses
                    // establish its complete image without changing family.
                    let relation = pcurve
                        .curve()
                        .intersect_curve(boundary.curve())
                        .map_err(GeometryError::from)?;
                    if !relation.is_complete() {
                        return Ok(None);
                    }
                    for overlap in relation.overlaps() {
                        if !overlap.includes_start() || !overlap.includes_end() {
                            return Ok(None);
                        }
                        // A polynomial branch's overlap range is already in
                        // its own (abscissa) parameter.
                        let mut a = if pcurve.curve().polynomial_branch().is_some() {
                            let scalar = |parameter: &hypercurve::CurveParameter2| {
                                parameter
                                    .scalar()
                                    .cloned()
                                    .ok_or(GeometryError::UnrepresentableParameter)
                            };
                            hypercurve::ParamRange::new(
                                scalar(overlap.first_range().start())?,
                                scalar(overlap.first_range().end())?,
                            )
                        } else {
                            crate::geometry::native_span_parameter_range(
                                pcurve.curve(),
                                overlap.first_span_index(),
                                overlap.first_range(),
                            )?
                        };
                        let mut b = crate::geometry::native_span_parameter_range(
                            boundary.curve(),
                            overlap.second_span_index(),
                            overlap.second_range(),
                        )?;
                        if order(a.start(), a.end())? == std::cmp::Ordering::Greater {
                            a = hypercurve::ParamRange::new(a.end().clone(), a.start().clone());
                            b = hypercurve::ParamRange::new(b.end().clone(), b.start().clone());
                        }
                        overlaps.push(ArenaBoundaryTraceOverlap {
                            edge_use: edge_use_id,
                            trace_range: a,
                            boundary_range: b,
                            native_overlap: Some(overlap.clone()),
                        });
                    }
                }
            }
        }
        let mut ordered = Vec::<ArenaBoundaryTraceOverlap>::with_capacity(overlaps.len());
        for overlap in overlaps {
            let mut index = ordered.len();
            while index > 0
                && order(
                    overlap.trace_range.start(),
                    ordered[index - 1].trace_range.start(),
                )? == std::cmp::Ordering::Less
            {
                index -= 1;
            }
            ordered.insert(index, overlap);
        }
        // Coverage runs over the trace's own parameter domain.
        let (mut covered, full) = match pcurve.curve().polynomial_branch() {
            Some(_) => {
                let domain = pcurve.curve().parameter_domain();
                match (domain.start().scalar(), domain.end().scalar()) {
                    (Some(start), Some(end)) => (start.clone(), end.clone()),
                    _ => return Ok(None),
                }
            }
            None => (Real::zero(), Real::one()),
        };
        for overlap in &ordered {
            if order(overlap.trace_range.start(), &covered)? == std::cmp::Ordering::Greater {
                return Ok(None);
            }
            if order(overlap.trace_range.end(), &covered)? == std::cmp::Ordering::Greater {
                covered = overlap.trace_range.end().clone();
            }
        }
        if order(&covered, &full)? != std::cmp::Ordering::Equal {
            return Ok(None);
        }

        // Parameter-space overlap alone does not certify the spatial trace.
        // Replay the existing complete image validator against this actual
        // face carrier, without trusting the intersection's cached proof flag.
        let curve = intersection.curve();
        let mut builder = ArenaModelBuilder::new();
        // Endpoint coordinates need not have independent scalar views. Keep
        // the selected point and its source relation for image validation.
        let start = builder.vertex(curve.point_at(curve.domain().start())?)?;
        let end = builder.vertex(curve.point_at(curve.domain().end())?)?;
        let curve_id = builder.curve(curve.clone())?;
        let edge = builder.edge(start, end, curve_id, curve.domain().clone())?;
        let pcurve = builder.pcurve(pcurve)?;
        let correspondence = match materialized.correspondence() {
            SurfacePcurveCorrespondence::Affine { scale, offset } => {
                ParameterCorrespondence::affine(scale.clone(), offset.clone())?
            }
            SurfacePcurveCorrespondence::AngularSweep { .. } => {
                ParameterCorrespondence::angular_sweep()
            }
        };
        let surface = builder.surface(
            self.surface(face.surface())
                .expect("validated surface")
                .clone(),
        )?;
        builder.validate_edge_image(edge, Direction::Forward, pcurve, correspondence, surface)?;
        Ok(Some(ArenaBoundarySurfaceTrace {
            source: self.clone(),
            intersection: intersection.clone(),
            operand,
            overlaps: ordered,
        }))
    }
}

#[cfg(test)]
mod tests;
