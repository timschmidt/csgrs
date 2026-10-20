//! Shared exact boundary-image proofs for validation and queries.

use crate::topology::*;

/// Scalar fast path for the signed triangle fan. `None` asks the caller to
/// integrate the retained boundary; it never substitutes selected coordinates.
pub(super) fn scalar_fan_six_volume<'a, E>(
    mut points: impl Iterator<Item = Result<&'a GeometricPoint3, E>>,
) -> Result<Option<Real>, E> {
    let Some(anchor) = points.next().transpose()? else {
        return Ok(Some(Real::zero()));
    };
    let Some(anchor) = anchor.try_coordinates() else {
        return Ok(None);
    };
    let Some(first) = points.next().transpose()? else {
        return Ok(Some(Real::zero()));
    };
    let Some(mut first) = first.try_coordinates() else {
        return Ok(None);
    };
    let mut sum = Real::zero();
    for second in points {
        let Some(second) = second?.try_coordinates() else {
            return Ok(None);
        };
        sum += Vector3::from(anchor.clone())
            .dot(&Vector3::from(first.clone()).cross(&Vector3::from(second.clone())));
        first = second;
    }
    Ok(Some(sum))
}

/// Divergence theorem for P(s,t) = origin + u*s + v*t:
/// 6V = 2 * (origin dot (u cross v)) * signed_UV_area.
/// The boundary winding includes face orientation and hole subtraction.
pub(super) fn planar_wire_six_volume(
    surface: &Surface,
    path: &CurvePath2,
) -> Result<Real, GeometryError> {
    let Some(origin) = surface.plane_origin() else {
        return Err(GeometryError::UnsupportedMeasurement);
    };
    let (u, v) = surface
        .plane_directions()
        .expect("plane origin implies plane directions");
    Ok(Real::from(2)
        * Vector3::from(origin.clone()).dot(&u.cross(v))
        * curve_path_signed_area(path)?)
}

/// Consecutive intervals of one source cover their union exactly once. Keep
/// the original topology; reuse a simpler image for its region/area proofs.
pub(super) fn coalesce_restrictions(input: Vec<Curve2>) -> Result<Vec<Curve2>, GeometryError> {
    if !input
        .iter()
        .any(|curve| curve.source_restriction().is_some() || curve.analytic_parallel().is_some())
    {
        return Ok(input);
    }
    let mut curves = Vec::with_capacity(input.len());
    for curve in input {
        if let Some(last) = curves.last_mut()
            && let Some(joined) = join_source_restrictions(last, &curve)?
        {
            *last = joined;
        } else {
            curves.push(curve);
        }
    }
    while curves.len() > 1 {
        let Some(joined) = join_source_restrictions(curves.last().expect("nonempty"), &curves[0])?
        else {
            break;
        };
        curves.pop();
        curves[0] = joined;
    }
    Ok(curves)
}

/// Exact subdivision identity: adjacent oriented intervals of the same authored
/// source cover their union once. This supplies validation evidence, never a
/// geometry repair or a topology edit.
fn join_source_restrictions(
    first: &Curve2,
    second: &Curve2,
) -> Result<Option<Curve2>, GeometryError> {
    if let (Some(first_view), Some(second_view)) =
        (first.analytic_parallel(), second.analytic_parallel())
    {
        if first_view.parallel() != second_view.parallel()
            || first_view.is_reversed() != second_view.is_reversed()
            || first
                .parameter_endpoints()
                .1
                .compare(second.parameter_endpoints().0)?
                != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }
        let (Some(start), Some(end)) = (
            first.parameter_endpoints().0.as_bezier_parameter(),
            second.parameter_endpoints().1.as_bezier_parameter(),
        ) else {
            return Ok(None);
        };
        let joined = match Curve2::try_analytic_parallel(
            first_view.parallel().clone(),
            hypercurve::BezierParameterRange2::try_new(start.clone(), end.clone())?,
        ) {
            Ok(joined) => joined,
            // Adjacent regular pieces can meet at a cusp. Failure to prove
            // their joined range leaves both original pieces in the proof.
            Err(hypercurve::ExactCurveError::Blocked(blocker))
                if blocker.reason() != hypercurve::UncertaintyReason::BudgetExhausted =>
            {
                return Ok(None);
            }
            Err(hypercurve::ExactCurveError::Invalid {
                cause: hypercurve::CurveError::Topology(_),
                ..
            }) => return Ok(None),
            Err(error) => return Err(error.into()),
        };
        return Ok(joined.has_certified_injective_image()?.then_some(joined));
    }
    let (Some((source, first_range, reversed)), Some((other, second_range, other_reversed))) =
        (first.source_restriction(), second.source_restriction())
    else {
        return Ok(None);
    };
    if source != other || reversed != other_reversed {
        return Ok(None);
    }
    // Removing an internal boundary from a simplicity proof must not hide
    // a self-contact inside the reassembled source. Lines are injective by
    // construction. Other sources need a complete image certificate, including
    // exact span joins and a common monotone coordinate for multi-span splines.
    if !matches!(source.geometry(), Some(CurveGeometry2::Line(_)))
        && !source.has_certified_injective_image()?
    {
        return Ok(None);
    }
    if first
        .parameter_endpoints()
        .1
        .compare(second.parameter_endpoints().0)
        .map_err(GeometryError::from)?
        != std::cmp::Ordering::Equal
    {
        return Ok(None);
    }
    let (start, end) = if reversed {
        (second_range.start(), first_range.end())
    } else {
        (first_range.start(), second_range.end())
    };
    let range = hypercurve::CurveParameterRange2::try_new(start.clone(), end.clone())
        .map_err(GeometryError::from)?;
    let joined = source.restrict_source(range).map_err(GeometryError::from)?;
    Ok(Some(if reversed {
        joined.reversed().map_err(GeometryError::from)?
    } else {
        joined
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hypercurve::CurveParameterRange2;

    #[test]
    fn parallel_reassembly_preserves_direction_source_and_adjacency() {
        let parallel = hypercurve::QuadraticBezier2::new(
            CurvePoint2::from_values(0, 0),
            CurvePoint2::from_values(2, 2),
            CurvePoint2::from_values(4, 0),
        )
        .parallel_left(Real::one())
        .unwrap();
        let curve = Curve2::try_analytic_parallel(
            parallel,
            hypercurve::BezierParameterRange2::try_new(
                hypercurve::BezierParameter2::Exact(0.into()),
                hypercurve::BezierParameter2::Exact(1.into()),
            )
            .unwrap(),
        )
        .unwrap();
        for curve in [curve.clone(), curve.reversed().unwrap()] {
            let half = (Real::one() / Real::from(2)).unwrap();
            let (a, b) = curve.split_at(half.into()).unwrap();
            let joined = coalesce_restrictions(vec![a.clone(), b.clone()]).unwrap();
            assert_eq!(joined.len(), 1);
            assert_eq!(
                joined[0].analytic_parallel().unwrap().parallel(),
                curve.analytic_parallel().unwrap().parallel()
            );
            assert_eq!(joined[0].parameter_endpoints(), curve.parameter_endpoints());
            assert_eq!(
                coalesce_restrictions(vec![b.clone(), a.reversed().unwrap()])
                    .unwrap()
                    .len(),
                2
            );
        }
    }

    #[test]
    fn parallel_reassembly_keeps_an_interior_cusp_split() {
        let half = (Real::one() / Real::from(2)).unwrap();
        let parallel = hypercurve::QuadraticBezier2::new(
            CurvePoint2::from_values(0, 0),
            CurvePoint2::new(half, 0.into()),
            CurvePoint2::from_values(1, 1),
        )
        .parallel_left(Real::one())
        .unwrap();
        let analysis = parallel
            .singularity_analysis(&CurveParameterRange2::unit())
            .unwrap();
        let [cusp] = analysis.parallel_cusps() else {
            panic!("one cusp")
        };
        let parts = [
            (hypercurve::BezierParameter2::Exact(0.into()), cusp.clone()),
            (cusp.clone(), hypercurve::BezierParameter2::Exact(1.into())),
        ]
        .into_iter()
        .map(|(a, b)| {
            Curve2::try_analytic_parallel(
                parallel.clone(),
                hypercurve::BezierParameterRange2::try_new(a, b).unwrap(),
            )
            .unwrap()
        })
        .collect();
        assert_eq!(coalesce_restrictions(parts).unwrap().len(), 2);
    }

    #[test]
    fn reassembly_does_not_hide_a_source_without_injectivity_evidence() {
        let uv = |x, y| CurvePoint2::new(Real::from(x), Real::from(y));
        let source = Curve2::from(
            RationalBezier2::try_new(
                vec![uv(0, 0), uv(2, 3), uv(-2, 3), uv(0, 0)],
                vec![Real::one(); 4],
            )
            .unwrap(),
        );
        let half = (Real::one() / Real::from(2)).unwrap();
        let first = source
            .restrict_source(
                CurveParameterRange2::try_new(Real::zero().into(), half.clone().into()).unwrap(),
            )
            .unwrap();
        let second = source
            .restrict_source(
                CurveParameterRange2::try_new(half.into(), Real::one().into()).unwrap(),
            )
            .unwrap();
        assert_eq!(coalesce_restrictions(vec![first, second]).unwrap().len(), 2);
    }
}
