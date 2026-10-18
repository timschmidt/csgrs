//! Finite plane contacts with retained source parameters and complete components.

use super::*;
use crate::ParameterRange;
use hypercurve::{
    Axis2, CurveIntersectionPairBlockerKind2, CurveParameterSet2, CurvePointLocations2, Similarity2,
};
use hyperreal::RealSign;

pub(super) fn intersect_rational_bezier_plane(
    curve: &Curve3,
    bezier: &RationalBezier3,
    plane: &PlaneSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let coefficients = bezier
        .control_points
        .iter()
        .zip(&bezier.weights)
        .map(|(point, weight)| weight * normal.dot(&(point - &plane.origin)))
        .collect::<Vec<_>>();
    let roots = match coefficients.as_slice() {
        [start, end] => Some(quadratic_roots(Real::zero(), end - start, start.clone())?),
        [start, middle, end] => Some(quadratic_roots(
            start - Real::from(2) * middle + end,
            Real::from(2) * (middle - start),
            start.clone(),
        )?),
        _ => None,
    };
    if let Some(roots) = roots {
        return match roots {
            QuadraticRoots::None => Ok(CurveSurfaceIntersection::None),
            QuadraticRoots::All => Ok(CurveSurfaceIntersection::Contained),
            QuadraticRoots::Isolated(roots) => isolated_line_parameters(curve, roots),
        };
    }
    let mut contained = true;
    for coefficient in &coefficients {
        if !is_zero(coefficient)? {
            contained = false;
            break;
        }
    }
    if contained {
        return Ok(CurveSurfaceIntersection::Contained);
    }
    // Positive native weights certify a nonzero denominator on the unit domain.
    // Reuse the planar owner's complete incidence/root authority for the
    // homogeneous numerator. At its zeros, N'=W*f', so positive W also preserves
    // the first-derivative classification without differentiating a quotient.
    let count = coefficients.len();
    let projected = Curve2::from(RationalBezier2::try_new(
        coefficients
            .into_iter()
            .map(|value| CurvePoint2::new(value, Real::zero()))
            .collect(),
        vec![Real::one(); count],
    )?);
    let mut points = Vec::new();
    match projected.point_locations(&CurvePoint2::new(Real::zero(), Real::zero()).into())? {
        CurvePointLocations2::EntireCurve => return Ok(CurveSurfaceIntersection::Contained),
        CurvePointLocations2::Locations(locations) => {
            for location in locations {
                let selected = location.parameter()?;
                let derivative = projected.derivatives_at(&selected, 1)?;
                let kind = classify_derivative(derivative[0].coordinate_sign(Axis2::X)?);
                let parameter = Parameter::from_curve(&selected)?;
                points.push(CurveSurfacePoint {
                    point: curve.point_at(&parameter)?,
                    parameter,
                    kind,
                });
            }
        }
    }
    assemble(curve, points, Vec::new())
}

pub(super) fn intersect_nurbs_plane(
    curve: &Curve3,
    nurbs: &NurbsCurve3,
    plane: &PlaneSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    for (segment, domain) in decompose_nurbs_into_bezier_segments(nurbs)? {
        let CurveGeometry3::RationalBezier(bezier) = &segment.data.geometry else {
            unreachable!("NURBS decomposition produces rational Bézier segments");
        };
        match intersect_rational_bezier_plane(&segment, bezier, plane)? {
            CurveSurfaceIntersection::None => {}
            CurveSurfaceIntersection::Points(local_points) => {
                let span = domain.end() - domain.start();
                for local in local_points {
                    let parameter = local.parameter.affine_image(&span, domain.start())?;
                    // Native decomposition already proves the represented
                    // point image. Preserve that low-degree path and its exact
                    // form; selected images must retain the original NURBS.
                    let point = if parameter.try_real().is_some() {
                        local.point
                    } else {
                        curve.point_at(&parameter)?
                    };
                    points.push(CurveSurfacePoint {
                        point,
                        parameter,
                        kind: local.kind,
                    });
                }
            }
            CurveSurfaceIntersection::Contained => overlaps.push(domain.into()),
            CurveSurfaceIntersection::Overlap(_) | CurveSurfaceIntersection::Components(_) => {
                unreachable!(
                    "one rational plane equation has isolated roots or is identically zero"
                )
            }
        }
    }
    assemble(curve, points, overlaps)
}

pub(super) fn intersect_planar_image_plane(
    curve: &Curve3,
    image: &PlanarCurve3,
    plane: &PlaneSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let a = normal.dot(&image.u);
    let b = normal.dot(&image.v);
    let c = normal.dot(&(&image.origin - &plane.origin));
    // A stored definition alone does not prove that a signed-weight source has
    // no poles. Certify the consumed interval before publishing even containment.
    let source = image.finite_source()?;
    let bounds = source.bounds()?;
    let a_zero = is_zero(&a)?;
    let b_zero = is_zero(&b)?;
    if a_zero && b_zero {
        return Ok(if is_zero(&c)? {
            CurveSurfaceIntersection::Contained
        } else {
            CurveSurfaceIntersection::None
        });
    }
    // Cover every possible source contact by a finite exact line. The added
    // unit extends a conservative enclosure; it is not a coincidence tolerance.
    let endpoints = if !a_zero {
        [bounds.min_y() - Real::one(), bounds.max_y() + Real::one()].map(|y| {
            Ok(CurvePoint2::new(
                ((-&b * &y - &c) / &a).map_err(GeometryError::Numeric)?,
                y,
            ))
        })
    } else {
        [bounds.min_x() - Real::one(), bounds.max_x() + Real::one()].map(|x| {
            Ok(CurvePoint2::new(
                x,
                ((-&c) / &b).map_err(GeometryError::Numeric)?,
            ))
        })
    };
    let [start, end]: [GeometryResult<CurvePoint2>; 2] = endpoints;
    let line = Curve2::from(LineSeg2::try_new(start?, end?)?);
    let intersection = source.intersect_curve(&line)?;
    if let Some(blocker) = intersection.blockers().first() {
        return Err(match blocker.kind() {
            CurveIntersectionPairBlockerKind2::Uncertain(reason) => {
                GeometryError::PlanarClassificationUnresolved(*reason)
            }
            CurveIntersectionPairBlockerKind2::IncompleteReplay
            | CurveIntersectionPairBlockerKind2::SharedComponent => {
                GeometryError::UnsupportedIntersection
            }
        });
    }
    let inverse = (Real::one() / (&image.source_end - &image.source_start))
        .map_err(GeometryError::Numeric)?;
    let offset = -&image.source_start * &inverse;
    let transport = |parameter: &hypercurve::CurveParameter2| {
        Parameter::from_curve(parameter)?.affine_image(&inverse, &offset)
    };
    let domain = ParameterRange::from(curve.domain());
    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    let mut derivative_image = None;
    for contact in intersection.contacts() {
        let source_parameter = contact.first().parameter()?;
        let parameter = transport(&source_parameter)?;
        if !domain.contains(&parameter)? {
            continue;
        }
        let kind = match contact.tangent_cross_sign() {
            Some(sign)
                if !matches!(
                    source.family(),
                    CurveFamily2::Nurbs | CurveFamily2::PolynomialBSpline
                ) =>
            {
                classify_derivative(sign)
            }
            _ => {
                // Lack of a transverse certificate is not a tangency proof.
                // A deduplicated spline contact may name only one incident
                // span, so inspect both sides of its authored joint as well.
                // An exact similarity puts a*x+b*y+c on the first axis while
                // preserving the owner's source chart and parallel authority.
                if derivative_image.is_none() {
                    let transform = Similarity2::try_from_real_affine(
                        a.clone(),
                        b.clone(),
                        -&b,
                        a.clone(),
                        c.clone(),
                        Real::zero(),
                    )?;
                    derivative_image = Some(source.transform_similarity(&transform)?);
                }
                let projected = derivative_image
                    .as_ref()
                    .expect("initialized derivative image");
                let mut classification = CurveSurfaceContactKind::Simple;
                for side in [
                    hypercurve::CurveParameterSide2::Left,
                    hypercurve::CurveParameterSide2::Right,
                ] {
                    let derivative = projected.derivatives_at_side(&source_parameter, 1, side)?;
                    if derivative[0].coordinate_sign(Axis2::X)? == RealSign::Zero {
                        classification = CurveSurfaceContactKind::Repeated;
                    }
                }
                classification
            }
        };
        points.push(CurveSurfacePoint {
            point: curve.point_at(&parameter)?,
            parameter,
            kind,
        });
    }
    let plane_constant = normal.dot(&Vector3::from(plane.origin.clone()));
    let boundary_point = |parameter: &Parameter| -> GeometryResult<GeometricPoint3> {
        let point = curve.point_at(parameter)?;
        if point.compare_linear_form(&normal, &plane_constant)? != Ordering::Equal {
            return Err(GeometryError::UnsupportedIntersection);
        }
        Ok(point)
    };
    let mut append_range =
        |index, range: &hypercurve::CurveParameterRange2| -> GeometryResult<()> {
            let first = transport(&source.parameter_from_span(index, range.start())?)?;
            let second = transport(&source.parameter_from_span(index, range.end())?)?;
            let (mut start, mut end) = if first.compare(&second)? == Ordering::Greater {
                (second, first)
            } else {
                (first, second)
            };
            if start.compare(domain.start())? == Ordering::Less {
                start = domain.start().clone();
            }
            if end.compare(domain.end())? == Ordering::Greater {
                end = domain.end().clone();
            }
            match start.compare(&end)? {
                Ordering::Less => {
                    // A promoted span's closed range may end at a discontinuous
                    // authored spline knot. Preserve the owner's boundary
                    // blocker instead of publishing an unconsumable endpoint.
                    boundary_point(&start)?;
                    boundary_point(&end)?;
                    overlaps.push(ParameterRange::new(start, end)?);
                }
                Ordering::Equal => points.push(CurveSurfacePoint {
                    point: boundary_point(&start)?,
                    parameter: start,
                    kind: CurveSurfaceContactKind::Boundary,
                }),
                Ordering::Greater => {}
            }
            Ok(())
        };
    for overlap in intersection.overlaps() {
        if !overlap.includes_start() || !overlap.includes_end() {
            return Err(GeometryError::UnsupportedIntersection);
        }
        append_range(overlap.first_span_index(), overlap.first_range())?;
    }
    for component in intersection.parameter_components() {
        match component.first_parameters() {
            CurveParameterSet2::Range(range) => append_range(component.first_span_index(), range)?,
            CurveParameterSet2::Single(_) => {
                // The other operand is a noncollapsed line, so a Cartesian
                // range component cannot have only one first-operand parameter.
                return Err(GeometryError::UnsupportedIntersection);
            }
        }
    }
    assemble(curve, points, overlaps)
}

fn is_zero(value: &Real) -> GeometryResult<bool> {
    Ok(decided_order(compare_reals(
        value,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? == Ordering::Equal)
}

fn classify_derivative(sign: RealSign) -> CurveSurfaceContactKind {
    if sign == RealSign::Zero {
        CurveSurfaceContactKind::Repeated
    } else {
        CurveSurfaceContactKind::Simple
    }
}

/// Normalize in the original source chart. Distinct visits to one spatial
/// point remain distinct parameters. Coincident ranges own their endpoints.
fn assemble(
    curve: &Curve3,
    points: Vec<CurveSurfacePoint>,
    overlaps: Vec<ParameterRange>,
) -> GeometryResult<CurveSurfaceIntersection> {
    let mut sorted = Vec::<ParameterRange>::new();
    for range in overlaps {
        let mut index = sorted.len();
        while index > 0 && range.start().compare(sorted[index - 1].start())? == Ordering::Less {
            index -= 1;
        }
        sorted.insert(index, range);
    }
    let mut ranges = Vec::<ParameterRange>::new();
    for range in sorted {
        if let Some(last) = ranges.last_mut()
            && range.start().compare(last.end())? != Ordering::Greater
        {
            if range.end().compare(last.end())? == Ordering::Greater {
                *last = ParameterRange::new(last.start().clone(), range.end().clone())?;
            }
        } else {
            ranges.push(range);
        }
    }
    let mut isolated = Vec::<CurveSurfacePoint>::new();
    'point: for point in points {
        for range in &ranges {
            if range.contains(&point.parameter)? {
                continue 'point;
            }
        }
        let mut index = isolated.len();
        while index > 0 {
            match point.parameter.compare(&isolated[index - 1].parameter)? {
                Ordering::Greater => break,
                Ordering::Less => index -= 1,
                Ordering::Equal => {
                    if point.kind == CurveSurfaceContactKind::Repeated {
                        isolated[index - 1].kind = CurveSurfaceContactKind::Repeated;
                    }
                    continue 'point;
                }
            }
        }
        isolated.insert(index, point);
    }
    if ranges.is_empty() {
        return Ok(if isolated.is_empty() {
            CurveSurfaceIntersection::None
        } else {
            CurveSurfaceIntersection::Points(isolated)
        });
    }
    if ranges.len() == 1 && isolated.is_empty() {
        let range = ranges.pop().expect("one normalized range");
        let domain = ParameterRange::from(curve.domain());
        return Ok(
            if range.start().compare(domain.start())? == Ordering::Equal
                && range.end().compare(domain.end())? == Ordering::Equal
            {
                CurveSurfaceIntersection::Contained
            } else {
                CurveSurfaceIntersection::Overlap(range)
            },
        );
    }
    Ok(CurveSurfaceIntersection::Components(
        CurveSurfaceComponents {
            points: isolated,
            overlaps: ranges,
        },
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_clearance_requires_two_independent_planes() {
        let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
        // x=(2t-1)^2 has a zero only where y=2t(1-t) is positive.
        // The control hull crosses the axis-containing separating plane.
        let quadratic = Curve3::rational_bezier(
            vec![
                Point3::new(1.into(), 0.into(), 0.into()),
                Point3::new((-1).into(), 1.into(), 1.into()),
                Point3::new(1.into(), 0.into(), 2.into()),
            ],
            vec![Real::one(); 3],
        )
        .unwrap();
        assert!(
            quadratic
                .is_strictly_off_axis(&Point3::origin(), &Vector3::z())
                .unwrap()
        );
        // At the selected y=0 root sqrt(1/2), x=(2t-1)^2+1/4 is positive.
        let selected = Curve3::rational_bezier(
            [q(5, 4), q(-1, 12), q(-1, 12), q(5, 4)]
                .into_iter()
                .zip([0, -1, -2, 3])
                .zip(0..4)
                .map(|((x, y), z)| Point3::new(x, y.into(), z.into()))
                .collect(),
            vec![Real::one(); 4],
        )
        .unwrap();
        assert!(
            selected
                .is_strictly_off_axis(&Point3::origin(), &Vector3::z())
                .unwrap()
        );
        // x=2t^2-1 and y=t(2t^2-1) share that selected interior root.
        let crossing = Curve3::rational_bezier(
            [q(-1, 1), q(-1, 1), q(-1, 3), q(1, 1)]
                .into_iter()
                .zip([q(0, 1), q(-1, 3), q(-2, 3), q(1, 1)])
                .zip(0..4)
                .map(|((x, y), z)| Point3::new(x, y, z.into()))
                .collect(),
            vec![Real::one(); 4],
        )
        .unwrap();
        assert!(
            !crossing
                .is_strictly_off_axis(&Point3::origin(), &Vector3::z())
                .unwrap()
        );
    }
}
