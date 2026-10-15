//! Shared exact boundary-image proofs for validation and queries.

use crate::topology::*;

/// Consecutive intervals of one source cover their union exactly once. Keep
/// the original topology; reuse a simpler image for its region/area proofs.
pub(super) fn coalesce_restrictions(input: Vec<Curve2>) -> Result<Vec<Curve2>, GeometryError> {
    if !input
        .iter()
        .any(|curve| curve.source_restriction().is_some())
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
    // construction. Other sources need one certified injective Bezier span.
    if !matches!(source.geometry(), Some(CurveGeometry2::Line(_))) {
        let fragments = source.native_bezier_fragments()?;
        if fragments.len() != 1 || !fragments[0].has_certified_injective_axis()? {
            return Ok(None);
        }
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
