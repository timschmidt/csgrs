//! Exact clamped spline records, retaining authored knots and weights.
use super::*;
use hyperbrep::Curve3;

pub(super) fn curve(out: &mut Output, curve: &Curve3, scale: &Real) -> Result<u64, WriteError> {
    let endpoint_knots;
    let (degree, controls, weights, knots) = if let Some(data) = curve.rational_bezier_geometry() {
        let degree = data.control_points.len() - 1;
        endpoint_knots = [Real::zero(), Real::one()];
        (degree, data.control_points, data.weights, None)
    } else if let Some(data) = curve.nurbs_geometry() {
        endpoint_knots = [Real::zero(), Real::one()];
        (
            data.degree,
            data.control_points,
            data.weights,
            Some(data.knots),
        )
    } else {
        return Err(out
            .site
            .error(WriteErrorKind::UnsupportedCurve(curve.kind())));
    };
    let mut points = Vec::new();
    for control in controls {
        let coordinates = geometry::converted(control, scale, out)?;
        let id = geometry::point(out, &coordinates)?;
        push(&mut points, id, out.site)?;
    }
    let mut distinct: Vec<(&Real, usize)> = Vec::new();
    if let Some(knots) = knots {
        for knot in knots {
            if let Some((previous, count)) = distinct.last_mut()
                && geometry::order(previous, knot, out.site)? == std::cmp::Ordering::Equal
            {
                *count += 1;
                continue;
            }
            push(&mut distinct, (knot, 1), out.site)?;
        }
    } else {
        for knot in &endpoint_knots {
            push(&mut distinct, (knot, degree + 1), out.site)?;
        }
    }
    out.complex_entity(|out| {
        out.put("BOUNDED_CURVE()B_SPLINE_CURVE(")?;
        out.put(&degree.to_string())?;
        out.put(",")?;
        out.references(&points)?;
        // Unknown flags describe the full carrier, not only its topological trim.
        out.put(",.UNSPECIFIED.,.U.,.U.)B_SPLINE_CURVE_WITH_KNOTS((")?;
        for (index, (_, count)) in distinct.iter().enumerate() {
            if index != 0 {
                out.put(",")?;
            }
            out.put(&count.to_string())?;
        }
        out.put("),(")?;
        for (index, (knot, _)) in distinct.iter().enumerate() {
            if index != 0 {
                out.put(",")?;
            }
            out.number(&geometry::rational(knot, out)?)?;
        }
        out.put("),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE((")?;
        for (index, weight) in weights.iter().enumerate() {
            if index != 0 {
                out.put(",")?;
            }
            out.number(&geometry::rational(weight, out)?)?;
        }
        out.put("))REPRESENTATION_ITEM('')")
    })
}
