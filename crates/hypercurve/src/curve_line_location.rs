//! Native line inverse queries retain the selected point's affine field.

use super::*;

pub(super) fn selected_point_locations(
    curve: &Curve2,
    point: &CurvePoint2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurvePointLocations2>> {
    let source = curve
        .source_restriction()
        .map_or(curve, |(source, _, _)| source);
    let Some(CurveGeometry2::Line(line)) = source.geometry() else {
        return Ok(None);
    };
    let invalid =
        |cause| ExactCurveError::invalid(CurveOperation2::Intersection, curve.family(), cause);
    let decided = |result: CurveResult<Classification<std::cmp::Ordering>>| {
        ExactCurveError::decided(CurveOperation2::Intersection, result.map_err(invalid)?)
    };
    let empty = || Some(CurvePointLocations2::Locations(Vec::new()));
    let (dx, dy) = line.delta();
    // The original support equation is required before an affine coordinate
    // becomes an inverse parameter. Coordinate agreement alone is insufficient.
    if !decided(crate::bezier_offset::point_affine_form_order_to_real(
        point,
        &(-&dy),
        &dx,
        &(-&dy * line.start().x() + &dx * line.start().y()),
        policy,
    ))?
    .is_eq()
    {
        return Ok(empty());
    }
    let (a, b, origin) = if crate::classify::is_zero(&dx, policy) == Some(false) {
        (Real::one(), Real::zero(), line.start().x())
    } else if crate::classify::is_zero(&dy, policy) == Some(false) {
        (Real::zero(), Real::one(), line.start().y())
    } else {
        return Err(ExactCurveError::blocked(
            CurveOperation2::Intersection,
            curve.family(),
            UncertaintyReason::RealSign,
        ));
    };
    let inverse = (Real::one() / if a == Real::one() { &dx } else { &dy })
        .map_err(|error| invalid(error.into()))?;
    let parameter = ExactCurveError::decided(
        CurveOperation2::Intersection,
        crate::bezier_offset::point_affine_form_parameter(
            point,
            &[a * &inverse, b * &inverse, -origin * &inverse],
            policy,
        )
        .map_err(invalid)?,
    )?;
    let parameter =
        polynomial_curve_section::admit_line_parameter(curve, parameter, point, policy)?;
    Ok(Some(CurvePointLocations2::Locations(
        parameter
            .into_iter()
            .map(|parameter| {
                CurveLocation2::new(
                    0,
                    CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
                    parameter,
                )
            })
            .collect(),
    )))
}
