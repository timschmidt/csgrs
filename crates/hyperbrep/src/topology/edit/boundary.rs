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
        if !intersection.pcurve(operand).materializes_native_line() {
            return Ok(None);
        }
        let materialized = intersection.pcurve(operand).materialize()?;
        let pcurve = Pcurve::new(materialized.curve().clone());
        let Some(trace) = pcurve.line_segment() else {
            return Ok(None);
        };
        let order =
            |a: &Real, b: &Real| decided_model_order(compare_reals(a, b, crate::STRICT_PREDICATES));
        let mut overlaps = Vec::new();
        for wire in std::iter::once(outer).chain(face.inner().iter().copied()) {
            for &edge_use_id in self.wire(wire).expect("validated face wire").edge_uses() {
                let edge_use = self.edge_use(edge_use_id).expect("validated wire use");
                let boundary = self.pcurve(edge_use.pcurve()).expect("validated pcurve");
                let Some(boundary) = boundary.line_segment() else {
                    continue;
                };
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
                        });
                    }
                    LineLineIntersection::None | LineLineIntersection::Point { .. } => {}
                    // A missing overlap certificate is not evidence of
                    // coincidence. Leave it to the ordinary split validator.
                    LineLineIntersection::Uncertain { .. } => return Ok(None),
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
        let mut covered = Real::zero();
        for overlap in &ordered {
            if order(overlap.trace_range.start(), &covered)? == std::cmp::Ordering::Greater {
                return Ok(None);
            }
            if order(overlap.trace_range.end(), &covered)? == std::cmp::Ordering::Greater {
                covered = overlap.trace_range.end().clone();
            }
        }
        if order(&covered, &Real::one())? != std::cmp::Ordering::Equal {
            return Ok(None);
        }

        // Parameter-space overlap alone does not certify the spatial trace.
        // Replay the existing complete image validator against this actual
        // face carrier, without trusting the intersection's cached proof flag.
        let curve = intersection.curve();
        let mut builder = ArenaModelBuilder::new();
        let start = builder.vertex(curve.start()?.into())?;
        let end = builder.vertex(curve.end()?.into())?;
        let curve_id = builder.curve(curve.clone())?;
        let edge = builder.edge(start, end, curve_id, curve.domain().clone().into())?;
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
