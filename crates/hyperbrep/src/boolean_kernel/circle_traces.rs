//! Complete circular trace unions across different exact parameter frames.

use super::*;

pub(super) fn coalesce_planar_images(
    surface: &Surface,
    curves: Vec<SurfaceIntersectionCurve>,
) -> Result<Vec<SurfaceIntersectionCurve>, BooleanError> {
    let mut groups: Vec<Vec<SurfaceIntersectionCurve>> = Vec::new();
    for curve in curves {
        let mut matching = None;
        for (index, group) in groups.iter().enumerate() {
            if same_circle_support(group[0].curve(), curve.curve())? {
                matching = Some(index);
                break;
            }
        }
        match matching {
            Some(index) => groups[index].push(curve),
            None => groups.push(vec![curve]),
        }
    }
    let mut result = Vec::new();
    for group in groups {
        if group.len() == 1 {
            result.extend(group);
            continue;
        }
        let mut full = None;
        for (index, curve) in group.iter().enumerate() {
            let domain = curve.curve().domain();
            if exact_order(&(domain.end() - domain.start()), &Real::tau())? == Ordering::Equal {
                full = Some(index);
                break;
            }
        }
        if let Some(index) = full {
            // A complete angular period on this exact support contains every
            // other trace in the group, independently of phase or traversal.
            result.push(group[index].clone());
        } else if let Some(trace) = native_full_circle_union(surface, &group)? {
            result.push(trace);
        } else {
            result.extend(group);
        }
    }
    Ok(result)
}

fn same_circle_support(a: &Curve3, b: &Curve3) -> Result<bool, GeometryError> {
    let (Curve3ExactData::EllipseArc(a), Curve3ExactData::EllipseArc(b)) =
        (a.exact_data(), b.exact_data())
    else {
        return Ok(false);
    };
    Ok(a.circle
        && b.circle
        && points_exactly_equal(&a.center, &b.center)?
        && exact_order(&a.x_radius, &b.x_radius)? == Ordering::Equal
        && exact_order(
            &a.x.cross(&a.y).cross(&b.x.cross(&b.y)).norm_squared(),
            &Real::zero(),
        )? == Ordering::Equal)
}

fn native_full_circle_union(
    surface: &Surface,
    group: &[SurfaceIntersectionCurve],
) -> Result<Option<SurfaceIntersectionCurve>, GeometryError> {
    let mut start = group[0].curve().domain().start().clone();
    let mut end = group[0].curve().domain().end().clone();
    let mut common_chart = true;
    for trace in &group[1..] {
        if !circle_supports_equal(group[0].curve(), trace.curve())? {
            common_chart = false;
            break;
        }
        start = exact_min(&start, trace.curve().domain().start())?;
        end = exact_max(&end, trace.curve().domain().end())?;
    }
    // A common angular chart spanning less than a period cannot cover the
    // whole circle. This exact exclusion also preserves arbitrarily small
    // gaps without asking a more expensive native intersection to rediscover it.
    if common_chart && exact_order(&(end - start), &Real::tau())? == Ordering::Less {
        return Ok(None);
    }
    let Curve3ExactData::EllipseArc(reference) = group[0].curve().exact_data() else {
        return Ok(None);
    };
    let full = SurfaceIntersectionCurve::on_plane(
        Curve3::circle_arc(
            reference.center,
            reference.x,
            reference.y,
            reference.x_radius,
            Real::zero(),
            Real::tau(),
        )?,
        surface,
    )?;
    let target = full.first_pcurve().materialize()?;
    let mut ranges: Vec<hypercurve::ParamRange> = Vec::new();
    for source in group {
        let projected = SurfaceIntersectionCurve::on_plane(source.curve().clone(), surface)?;
        let projected = projected.first_pcurve().materialize()?;
        let relation = target.curve().intersect_curve(projected.curve())?;
        if !relation.is_complete() {
            return Ok(None);
        }
        for overlap in relation.overlaps() {
            if !overlap.includes_start() || !overlap.includes_end() {
                return Ok(None);
            }
            let mut range = crate::geometry::native_span_parameter_range(
                target.curve(),
                overlap.first_span_index(),
                overlap.first_range(),
            )?;
            if exact_order(range.start(), range.end())? == Ordering::Greater {
                range = hypercurve::ParamRange::new(range.end().clone(), range.start().clone());
            }
            let mut index = ranges.len();
            while index > 0
                && exact_order(range.start(), ranges[index - 1].start())? == Ordering::Less
            {
                index -= 1;
            }
            ranges.insert(index, range);
        }
    }
    let mut covered = Real::zero();
    for range in ranges {
        if exact_order(range.start(), &covered)? == Ordering::Greater {
            return Ok(None);
        }
        covered = exact_max(&covered, range.end())?;
    }
    Ok((exact_order(&covered, &Real::one())? == Ordering::Equal).then_some(full))
}
