//! Exact periodic chart lifts for retained surface intersections.

use super::*;

pub(super) fn periodic_face_trace_lifts(
    model: &ArenaModel,
    face: FaceId,
    intersection: &SurfaceIntersectionCurve,
    operand: SurfaceIntersectionOperand,
) -> Result<Vec<SurfaceIntersectionCurve>, GeometryError> {
    let pcurve = intersection.pcurve(operand);
    let surface = face_surface(model, face);
    let periodic = |domain: &crate::SurfaceParameterDomain| {
        matches!(domain, crate::SurfaceParameterDomain::Periodic { .. })
    };
    if face_is_boundaryless(model, face)
        || !pcurve.is_iso_parameter_line()
        || (!periodic(surface.domain().u()) && !periodic(surface.domain().v()))
    {
        return Ok(vec![intersection.clone()]);
    }
    let region = planar_face_region(model, face)?;
    let bounds = match crate::error::classified_present(region.bounds())? {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => {
            // Clipping an unshifted trace could certify the wrong periodic
            // chart as empty. Lack of a chart bound is a blocker, not a proof
            // that no equivalent lift intersects this face.
            return Err(GeometryError::PlanarClassificationUnresolved(reason));
        }
    };
    let start = pcurve.point_at(intersection.curve().domain().start())?;
    let end = pcurve.point_at(intersection.curve().domain().end())?;
    let shifts = |domain: &crate::SurfaceParameterDomain,
                  a: &Real,
                  b: &Real,
                  min: &Real,
                  max: &Real|
     -> Result<Vec<Real>, GeometryError> {
        let crate::SurfaceParameterDomain::Periodic { period, .. } = domain else {
            return Ok(vec![Real::zero()]);
        };
        let lower = exact_min(a, b)?;
        let upper = exact_max(a, b)?;
        let first = ((min - &upper) / period)
            .map_err(GeometryError::Numeric)?
            .ceil_certified()
            .map_err(GeometryError::Numeric)?;
        let last = ((max - &lower) / period)
            .map_err(GeometryError::Numeric)?
            .floor_certified()
            .map_err(GeometryError::Numeric)?;
        let first = Real::new(hyperreal::Rational::from_bigint(first));
        let last = Real::new(hyperreal::Rational::from_bigint(last));
        // At most three copies intersect one supported period-wide chart.
        // This bounds enumeration independently of a distant authored origin.
        if exact_order(&(&last - &first), &Real::from(2))? == Ordering::Greater {
            return Err(GeometryError::UnsupportedIntersection);
        }
        let mut offsets = Vec::new();
        for step in 0..3 {
            let turn = &first + Real::from(step);
            if exact_order(&turn, &last)? == Ordering::Greater {
                break;
            }
            let offset = turn * period;
            let low = exact_max(&(&lower + &offset), min)?;
            let high = exact_min(&(&upper + &offset), max)?;
            let order = exact_order(&low, &high)?;
            if order == Ordering::Less
                || (order == Ordering::Equal && exact_order(&lower, &upper)? == Ordering::Equal)
            {
                offsets.push(offset);
            }
        }
        Ok(offsets)
    };
    let u = shifts(
        surface.domain().u(),
        &start.x,
        &end.x,
        bounds.min_x(),
        bounds.max_x(),
    )?;
    let v = shifts(
        surface.domain().v(),
        &start.y,
        &end.y,
        bounds.min_y(),
        bounds.max_y(),
    )?;
    let mut lifts = Vec::with_capacity(u.len() * v.len());
    for u in &u {
        for v in &v {
            lifts.push(intersection.translated_iso_pcurve(operand, u, v)?);
        }
    }
    Ok(lifts)
}
