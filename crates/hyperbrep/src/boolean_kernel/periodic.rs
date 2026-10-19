//! Exact periodic chart lifts for retained surface intersections.

use super::*;

/// Additional meridian cuts make every later finite latitude attach to a
/// boundary even on a single full-period cylinder face. They only partition
/// the existing material, like planar support cuts; original intersection
/// incidences remain in the graph and each new boundary is source-face owned.
pub(super) fn axial_support_traces(
    model: &ArenaModel,
    face: FaceId,
    curves: Vec<SurfaceIntersectionCurve>,
    operand: SurfaceIntersectionOperand,
) -> Result<Vec<SurfaceIntersectionCurve>, GeometryError> {
    let surface = face_surface(model, face);
    let region = planar_face_region(model, face)?;
    let bounds = match crate::error::classified_present(region.bounds())? {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(_) => return Ok(curves),
    };
    let mut result = Vec::new();
    for trace in curves {
        if trace.curve().kind() != crate::Curve3Kind::Line
            || !trace.pcurve(operand).is_iso_parameter_line()
        {
            result.push(trace);
            continue;
        }
        let a = trace
            .pcurve(operand)
            .point_at(trace.curve().domain().start())?
            .coordinates()
            .cloned()
            .ok_or(GeometryError::UnrepresentablePoint)?;
        let b = trace
            .pcurve(operand)
            .point_at(trace.curve().domain().end())?
            .coordinates()
            .cloned()
            .ok_or(GeometryError::UnrepresentablePoint)?;
        if exact_order(a.x(), b.x())? != Ordering::Equal {
            result.push(trace);
            continue;
        }
        let a = crate::Point2::new(a.x().clone(), bounds.min_y().clone());
        let b = crate::Point2::new(a.x.clone(), bounds.max_y().clone());
        let curve = Curve3::line(surface.evaluate_real(&a)?, surface.evaluate_real(&b)?)?;
        let pcurve = crate::Pcurve::new(Curve2::from(LineSeg2::try_new(
            hypercurve::Point2::new(a.x, a.y),
            hypercurve::Point2::new(b.x, b.y),
        )?));
        // On a native cylinder, constant u makes the entire v image affine.
        let support = SurfaceIntersectionCurve::from_certified_iso_boundary(
            curve,
            &pcurve,
            &Real::one(),
            &Real::zero(),
        )?
        .expect("a native constant-u cylinder line");
        let intervals = match retained_curve_face_intervals(
            model,
            face,
            support.pcurve(operand),
            support.curve(),
            None,
        )? {
            Classification::Decided(Some(intervals)) => intervals,
            Classification::Decided(None) | Classification::Uncertain(_) => {
                result.push(trace);
                continue;
            }
        };
        for interval in intervals {
            result.push(
                support
                    .subcurve(&interval.start, &interval.end)?
                    .with_boundary_contacts(
                        operand,
                        interval.start_contacts,
                        interval.end_contacts,
                    ),
            );
        }
    }
    Ok(result)
}

/// Keep the union of complete iso-line images in one authored face chart.
/// Subtract previously retained intervals from each trace; splitting preserves
/// the original spatial curve, both pcurves and their endpoint evidence. The
/// intersection graph separately retains every original source incidence.
pub(super) fn disjoint_iso_traces(
    curves: Vec<SurfaceIntersectionCurve>,
    operand: SurfaceIntersectionOperand,
) -> Result<Vec<SurfaceIntersectionCurve>, GeometryError> {
    fn interval(
        trace: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
    ) -> Result<Option<(bool, Real, Real, Real)>, GeometryError> {
        let pcurve = trace.pcurve(operand);
        if !pcurve.is_iso_parameter_line() {
            return Ok(None);
        }
        let a = pcurve
            .point_at(trace.curve().domain().start())?
            .coordinates()
            .cloned()
            .ok_or(GeometryError::UnrepresentablePoint)?;
        let b = pcurve
            .point_at(trace.curve().domain().end())?
            .coordinates()
            .cloned()
            .ok_or(GeometryError::UnrepresentablePoint)?;
        if exact_order(a.x(), b.x())? == Ordering::Equal {
            Ok(Some((
                true,
                a.x().clone(),
                exact_min(a.y(), b.y())?,
                exact_max(a.y(), b.y())?,
            )))
        } else if exact_order(a.y(), b.y())? == Ordering::Equal {
            Ok(Some((
                false,
                a.y().clone(),
                exact_min(a.x(), b.x())?,
                exact_max(a.x(), b.x())?,
            )))
        } else {
            Ok(None)
        }
    }
    fn subtrace(
        trace: &SurfaceIntersectionCurve,
        operand: SurfaceIntersectionOperand,
        u_constant: bool,
        lower: &Real,
        upper: &Real,
    ) -> Result<SurfaceIntersectionCurve, GeometryError> {
        let domain = trace.curve().domain();
        let a = trace
            .pcurve(operand)
            .point_at(domain.start())?
            .coordinates()
            .cloned()
            .ok_or(GeometryError::UnrepresentablePoint)?;
        let b = trace
            .pcurve(operand)
            .point_at(domain.end())?
            .coordinates()
            .cloned()
            .ok_or(GeometryError::UnrepresentablePoint)?;
        let (a, b) = if u_constant {
            (a.y().clone(), b.y().clone())
        } else {
            (a.x().clone(), b.x().clone())
        };
        let scale = ((domain.represented_end()? - domain.represented_start()?) / (&b - &a))
            .map_err(GeometryError::Numeric)?;
        let first = domain.represented_start()? + (lower - &a) * &scale;
        let second = domain.represented_start()? + (upper - &a) * scale;
        trace.subcurve(&exact_min(&first, &second)?, &exact_max(&first, &second)?)
    }
    let mut result = Vec::<SurfaceIntersectionCurve>::new();
    for curve in curves {
        let mut pieces = vec![curve];
        for previous in &result {
            let Some((axis, constant, lower, upper)) = interval(previous, operand)? else {
                continue;
            };
            let mut remaining = Vec::new();
            for piece in pieces {
                let Some((other_axis, other_constant, start, end)) = interval(&piece, operand)?
                else {
                    remaining.push(piece);
                    continue;
                };
                if axis != other_axis
                    || exact_order(&constant, &other_constant)? != Ordering::Equal
                    || exact_order(&start, &upper)? != Ordering::Less
                    || exact_order(&end, &lower)? != Ordering::Greater
                {
                    remaining.push(piece);
                    continue;
                }
                if exact_order(&start, &lower)? == Ordering::Less {
                    remaining.push(subtrace(&piece, operand, axis, &start, &lower)?);
                }
                if exact_order(&end, &upper)? == Ordering::Greater {
                    remaining.push(subtrace(&piece, operand, axis, &upper, &end)?);
                }
            }
            pieces = remaining;
            if pieces.is_empty() {
                break;
            }
        }
        result.extend(pieces);
    }
    Ok(result)
}

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
    let start = pcurve
        .point_at(intersection.curve().domain().start())?
        .coordinates()
        .cloned()
        .ok_or(GeometryError::UnrepresentablePoint)?;
    let end = pcurve
        .point_at(intersection.curve().domain().end())?
        .coordinates()
        .cloned()
        .ok_or(GeometryError::UnrepresentablePoint)?;
    let u = periodic_offsets(
        surface.domain().u(),
        start.x(),
        end.x(),
        bounds.min_x(),
        bounds.max_x(),
    )?;
    let v = periodic_offsets(
        surface.domain().v(),
        start.y(),
        end.y(),
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

/// Enumerate exact whole-period lifts intersecting one admitted trim chart.
pub(super) fn periodic_offsets(
    domain: &crate::SurfaceParameterDomain,
    a: &Real,
    b: &Real,
    min: &Real,
    max: &Real,
) -> Result<Vec<Real>, GeometryError> {
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
}
