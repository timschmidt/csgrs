//! Boundary traces on coincident analytic carriers with certified chart maps.

use super::*;

pub(super) fn common_parameter_boundary_traces(
    source_model: &ArenaModel,
    source_face: FaceId,
    target_model: &ArenaModel,
    target_face: FaceId,
    target_operand: SurfaceIntersectionOperand,
) -> Result<Classification<Option<Vec<SurfaceIntersectionCurve>>>, GeometryError> {
    let face = source_model
        .face(source_face)
        .expect("validated source face");
    let Some(outer) = face.outer() else {
        return Ok(Classification::Decided(None));
    };
    let mut traces = Vec::new();
    for wire in std::iter::once(outer).chain(face.inner().iter().copied()) {
        for &edge_use in source_model.wire(wire).expect("validated wire").edge_uses() {
            let edge_use = source_model.edge_use(edge_use).expect("validated edge use");
            let Some((scale, offset)) = edge_use.parameter_correspondence().affine_coefficients()
            else {
                return Ok(Classification::Decided(None));
            };
            let edge = source_model.edge(edge_use.edge()).expect("validated edge");
            let curve = source_model
                .curve(edge.curve())
                .expect("validated curve")
                .subcurve(
                    edge.domain()
                        .start()
                        .try_real()
                        .ok_or(GeometryError::UnrepresentableParameter)?,
                    edge.domain()
                        .end()
                        .try_real()
                        .ok_or(GeometryError::UnrepresentableParameter)?,
                )?;
            let pcurve = source_model
                .pcurve(edge_use.pcurve())
                .expect("validated pcurve");
            let Some(trace) = SurfaceIntersectionCurve::from_coincident_iso_boundary(
                curve,
                pcurve,
                scale,
                offset,
                face_surface(source_model, source_face),
                face_surface(target_model, target_face),
                target_operand,
            )?
            else {
                return Ok(Classification::Decided(None));
            };
            // Shift only by exact whole periods, retaining the original
            // spatial curve and the source face's actual chart in its slot.
            for trace in
                periodic_face_trace_lifts(target_model, target_face, &trace, target_operand)?
            {
                match retained_curve_face_intervals(
                    target_model,
                    target_face,
                    trace.pcurve(target_operand),
                    trace.curve(),
                    None,
                )? {
                    Classification::Decided(Some(intervals)) => {
                        for interval in intervals {
                            traces.push(
                                trace
                                    .subcurve(&interval.start, &interval.end)?
                                    .with_boundary_contacts(
                                        target_operand,
                                        interval.start_contacts,
                                        interval.end_contacts,
                                    ),
                            );
                        }
                    }
                    Classification::Decided(None) => return Ok(Classification::Decided(None)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
    }
    Ok(Classification::Decided(Some(traces)))
}
